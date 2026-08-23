#!/usr/bin/env python3
"""
PiperArmController: 封装机械臂笛卡尔空间运动控制。

双模式:
  - 直连模式 (use_service=False): 进程内 MoveIt，用于工作空间分析 (需要 launch 文件)
  - 服务模式 (use_service=True): 通过 ROS 服务控制真实机械臂 (可独立 rosrun)

用法:
    # 服务模式 (控制真实机械臂)
    controller = PiperArmController.from_yaml("upper", use_service=True)
    controller.move_to(0.4, 0.0, 0.3)

    # 直连模式 (工作空间分析)
    controller = PiperArmController(eye_position=(1.0, 0.0, 0.5), arm_group="arm")
    controller.move_to(0.4, 0.0, 0.3)
"""

import os
from pathlib import Path

# 抑制 MoveIt 规划 TIMED_OUT 刷屏 WARN (必须在 roscpp 初始化前设置)
_SCRIPT_DIR = Path(__file__).resolve().parent
_CONFIG = str(_SCRIPT_DIR / 'rosconsole_no_warn.config')
if os.path.exists(_CONFIG) and 'ROSCONSOLE_CONFIG_FILE' not in os.environ:
    os.environ['ROSCONSOLE_CONFIG_FILE'] = _CONFIG

import functools
import math
import sys
import threading
import time

import yaml
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_about_axis, quaternion_multiply

# 服务模式导入 (可能不存在，允许导入失败)
try:
    from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
    _HAS_SRV = True
except ImportError:
    _HAS_SRV = False

# 直连模式导入 (需要 moveit_commander)
try:
    import moveit_commander
    _HAS_MOVEIT = True
except ImportError:
    _HAS_MOVEIT = False


# ------------------------------------------------------------------
# 服务调用辅助函数 (参考 joint_moveit_ctrl.py)
# ------------------------------------------------------------------

# 服务调用超时 (秒): 防止 move_group 假死时调用方永久挂起
SRV_WAIT_TIMEOUT = 3.0   # wait_for_service 等桥接服务就绪
SRV_CALL_TIMEOUT = 10.0  # 单次调用上限, 需覆盖规划+执行全程 (速度 0.3 的最大行程 < 10s)


class ServiceCallTimeout(rospy.ServiceException):
    """服务调用硬超时 (子类: 现有 except ServiceException 依然捕获)。

    区别于普通服务错误: 超时意味着系统不健康 (move_group 假死/掉线)，
    搜索循环应整体中止而非继续尝试下一候选。
    """


def _wait_for_service(srv_name, timeout=SRV_WAIT_TIMEOUT):
    """带超时的服务等待。超时抛 ServiceException，与调用超时统一处理。"""
    try:
        rospy.wait_for_service(srv_name, timeout=timeout)
    except rospy.ROSException:
        raise rospy.ServiceException(
            f"service {srv_name} not available after {timeout:.1f}s")


def _call_srv_with_timeout(srv, req, timeout=SRV_CALL_TIMEOUT):
    """线程级硬超时服务调用。超时抛 ServiceException，由调用方统一处理。

    超时后 worker 线程仍在后台等待响应 (daemon)，与 _plan_with_timeout 同一取舍。
    """
    holder = {"result": None, "exception": None}

    def worker():
        try:
            holder["result"] = srv(req)
        except Exception as e:
            holder["exception"] = e

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    t.join(timeout=timeout)

    if t.is_alive():
        raise ServiceCallTimeout(f"service call timed out after {timeout:.1f}s")
    if holder["exception"] is not None:
        raise holder["exception"]
    return holder["result"]


def _call_joint_moveit_ctrl_arm(joint_states, max_velocity=0.5, max_acceleration=0.5,
                                planning_time=0.05, ns=""):
    srv_name = f"{ns}/joint_moveit_ctrl_arm" if ns else "joint_moveit_ctrl_arm"
    _wait_for_service(srv_name)
    srv = rospy.ServiceProxy(srv_name, JointMoveitCtrl)
    req = JointMoveitCtrlRequest()
    req.joint_states = joint_states
    req.gripper = 0.0
    req.max_velocity = max_velocity
    req.max_acceleration = max_acceleration
    if hasattr(req, 'planning_time'):
        req.planning_time = planning_time
    return _call_srv_with_timeout(srv, req)


def _call_joint_moveit_ctrl_endpose(x, y, z, qx, qy, qz, qw,
                                     max_velocity=0.5, max_acceleration=0.5,
                                     planning_time=0.05, ns=""):
    srv_name = f"{ns}/joint_moveit_ctrl_endpose" if ns else "joint_moveit_ctrl_endpose"
    _wait_for_service(srv_name)
    srv = rospy.ServiceProxy(srv_name, JointMoveitCtrl)
    req = JointMoveitCtrlRequest()
    req.joint_states = [0.0] * 6
    req.gripper = 0.0
    req.max_velocity = max_velocity
    req.max_acceleration = max_acceleration
    req.joint_endpose = [x, y, z, qx, qy, qz, qw]
    if hasattr(req, 'planning_time'):
        req.planning_time = planning_time
    return _call_srv_with_timeout(srv, req)


# ------------------------------------------------------------------
# PiperArmController
# ------------------------------------------------------------------

def _synchronized(method):
    """串行化公开机械臂操作 (可重入 RLock): 同一 controller 的
    move/查询/清理互斥，多线程共享一个实例是安全的。"""
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return wrapper


class PiperArmController:
    """机械臂末端运动控制器，支持法兰盘对眼位 (eye_position) 的 (yaw, pitch) 联合搜索。"""

    def __init__(
        self,
        eye_position=(1.0, 0.0, 0.5),
        arm_group="arm",
        can_port="",
        safety_bbox=None,
        obstacles=None,
        yaw_step=10.0,
        yaw_levels=3,
        pitch_step=5.0,
        pitch_range=20.0,
        planning_timeout=0.05,
        hard_timeout=0.5,
        velocity_scaling=1.0,
        acceleration_scaling=1.0,
        position_tolerance=0.005,
        orientation_tolerance=0.01,
        use_service=False,
        frame_id=None,
    ):
        """
        :param eye_position: 人眼在基座坐标系中的位置 (x, y, z) 米
        :param arm_group: MoveIt 规划组名称
        :param safety_bbox: 大空间限制 {'x': [min, max], 'y': [min, max], 'z': [min, max]} 或 None
        :param obstacles: 空间内小障碍物列表
        :param yaw_step: yaw 搜索步长 (度)
        :param yaw_levels: yaw 搜索层数
        :param pitch_step: pitch 搜索步长 (度)
        :param pitch_range: 理想 yaw 时的 pitch 搜索范围 (度)
        :param planning_timeout: MoveIt 软超时 (秒)
        :param hard_timeout: 线程级硬超时兜底 (秒)
        :param velocity_scaling: 最大速度比例 (0-1)
        :param acceleration_scaling: 最大加速度比例 (0-1)
        :param position_tolerance: 位置容差 (米)
        :param orientation_tolerance: 姿态容差 (弧度)
        :param use_service: True=通过 ROS 服务控制真实机械臂, False=进程内 MoveIt
        :param frame_id: 基座坐标系 (服务模式下必须指定，直连模式自动从 MoveIt 获取)
        """
        self.eye_position = eye_position
        self.safety_bbox = safety_bbox
        self.obstacles = obstacles if obstacles is not None else []
        self.yaw_step = yaw_step
        self.yaw_levels = yaw_levels
        self.pitch_step = pitch_step
        self.pitch_range = pitch_range
        self.planning_timeout = planning_timeout
        self.hard_timeout = hard_timeout
        self._use_service = use_service
        self._arm_group = arm_group
        self.can_port = can_port

        self._wall_names = ["safety_wall_floor", "safety_wall_ceiling",
                            "safety_wall_y_pos", "safety_wall_y_neg",
                            "safety_wall_x_pos", "safety_wall_x_neg"]
        self._walls_added = False

        # 可重入锁: print_current_pose 等组合方法会嵌套调用已加锁的查询方法
        self._lock = threading.RLock()
        # 超时弃置但仍在运行的规划线程 (见 _plan_with_timeout)
        self._orphan_threads = []
        # fast_solve 由模式派生: 服务模式由 from_yaml 加载偏移表, 直连/分析模式恒空
        self._offset_records = []

        if use_service:
            self._init_service_mode(frame_id or "dummy_link")
        else:
            self._init_inprocess_mode(arm_group, velocity_scaling, acceleration_scaling,
                                      position_tolerance, orientation_tolerance)

        if self.safety_bbox is not None:
            self._add_safety_walls()
        if self.obstacles:
            self._add_obstacles()

        mode = "service" if use_service else "in-process"
        if use_service:
            ns_prefix = f"{self.can_port}/" if self.can_port else ""
            rospy.set_param(f"{ns_prefix}joint_moveit_ctrl_server/planning_time", self.planning_timeout)
        rospy.loginfo(f"PiperArmController 已初始化 [{mode}]: arm='{arm_group}', "
                      f"eye=({eye_position[0]:.1f}, {eye_position[1]:.1f}, {eye_position[2]:.1f}), "
                      f"planning_timeout={self.planning_timeout}s, "
                      f"safety_bbox={safety_bbox}, obstacles_count={len(self.obstacles)}")

    # ------------------------------------------------------------------
    # 初始化: 服务模式
    # ------------------------------------------------------------------

    def _init_service_mode(self, frame_id):
        if not _HAS_SRV:
            raise RuntimeError("moveit_ctrl.srv 未找到，请确保 catkin_make 已完成且 setup.bash 已 source")

        self.scene = None  # 延迟初始化，避免阻塞在没有 move_group 的场景
        self.robot = None
        self.move_group = None
        self._frame_id = frame_id
        rospy.loginfo(f"服务模式已初始化: frame_id={frame_id}")

    def _ensure_scene(self):
        """延迟初始化 PlanningSceneInterface (超时 2s)，失败返回 None。

        PlanningSceneInterface 通过 ns 参数寻址 namespaced move_group 服务。
        返回 None 时调用方必须中止 — 不允许在无碰撞约束下运动。
        """
        if self.scene is not None:
            return self.scene

        if not _HAS_MOVEIT:
            rospy.logwarn("moveit_commander 未安装，无法使用 PlanningScene")
            return None

        moveit_commander.roscpp_initialize(sys.argv)

        holder = {"scene": None}

        def _init():
            try:
                holder["scene"] = moveit_commander.PlanningSceneInterface(ns=self.can_port)
            except Exception as e:
                holder["scene"] = e

        t = threading.Thread(target=_init, daemon=True)
        t.start()
        t.join(timeout=2.0)

        if t.is_alive():
            rospy.logwarn("PlanningSceneInterface 初始化超时 (move_group 可能未启动)")
            return None

        result = holder["scene"]
        if isinstance(result, Exception):
            rospy.logwarn(f"PlanningSceneInterface 初始化失败: {result}")
            return None

        self.scene = result
        rospy.loginfo("PlanningSceneInterface 延迟初始化成功")
        return self.scene

    # ------------------------------------------------------------------
    # 初始化: 直连模式
    # ------------------------------------------------------------------

    def _init_inprocess_mode(self, arm_group, velocity_scaling, acceleration_scaling,
                             position_tolerance, orientation_tolerance):
        if not _HAS_MOVEIT:
            raise RuntimeError("moveit_commander 未找到，请确保已安装 ros-noetic-moveit")

        ns = self.can_port  # 连接 namespaced move_group (e.g. /piper_lower/move_group)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander(ns=ns)

        # PlanningSceneInterface 需要 move_group 节点在运行，带超时以免永久挂起
        holder = {"scene": None}

        def _init_scene():
            try:
                holder["scene"] = moveit_commander.PlanningSceneInterface(ns=ns)
            except Exception as e:
                holder["scene"] = e

        t = threading.Thread(target=_init_scene, daemon=True)
        t.start()
        t.join(timeout=3.0)

        if t.is_alive():
            raise RuntimeError(
                "move_group 节点未运行，无法连接 PlanningScene。"
                "请先启动 demo.launch:\n"
                "  roslaunch piper_no_gripper_moveit demo.launch can_port:=piper_lower"
            )

        result = holder["scene"]
        if isinstance(result, Exception):
            raise RuntimeError(f"PlanningSceneInterface 初始化失败: {result}")

        self.scene = result
        self.move_group = moveit_commander.MoveGroupCommander(arm_group, ns=ns)
        self._frame_id = self.move_group.get_planning_frame()

        self.move_group.set_max_velocity_scaling_factor(velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acceleration_scaling)
        self.move_group.set_goal_position_tolerance(position_tolerance)
        self.move_group.set_goal_orientation_tolerance(orientation_tolerance)
        self.move_group.set_planning_time(self.planning_timeout)
        self.move_group.set_num_planning_attempts(1)

    # ------------------------------------------------------------------
    # 工厂方法
    # ------------------------------------------------------------------

    @classmethod
    def from_yaml(cls, yaml_path, use_service=False):
        """从 YAML 配置文件创建 PiperArmController 实例。

        支持短名称 (如 "upper" → cfg/piper_upper.yaml) 或完整路径。
        :param yaml_path: YAML 文件路径或短名称
        :param use_service: True=服务模式 (控制真实机械臂), False=直连模式
        """
        path = Path(yaml_path)
        if path.suffix not in ('.yaml', '.yml'):
            path = _SCRIPT_DIR / 'cfg' / f'piper_{yaml_path}.yaml'
        cfg = yaml.safe_load(path.read_text())

        arm = cfg.get('arm', {})
        can_port = arm.get('can_port', '')
        obstacles = cfg.get('obstacles', []) or []
        instance = cls(
            eye_position=tuple(arm.get('eye_position', (1.0, 0.0, 0.5))),
            arm_group=arm.get('group', 'arm'),
            can_port=can_port,
            safety_bbox=cfg.get('safety_bbox'),
            obstacles=obstacles,
            planning_timeout=arm.get('planning_timeout', 0.05),
            hard_timeout=arm.get('hard_timeout', 0.5),
            yaw_step=arm.get('yaw_step', 10.0),
            yaw_levels=arm.get('yaw_levels', 3),
            pitch_step=arm.get('pitch_step', 5.0),
            pitch_range=arm.get('pitch_range', 20.0),
            use_service=use_service,
        )
        instance.workspace_config = cfg.get('workspace_analysis', {})
        instance.yaml_path = str(path)
        # fast_solve: 仅服务模式 (直连/分析模式是偏移数据生产方, 必须纯螺旋)
        if use_service:
            instance._offset_records = instance._load_offset_records(path)
        return instance

    def _offset_file_path(self, yaml_path):
        """与 analyzer 输出命名一致: offsets_{yaml_stem}_{config_label}.txt"""
        stem = Path(yaml_path).stem          # e.g. piper_upper
        return _SCRIPT_DIR / 'reachable_range' / f'offsets_{stem}_{self.get_config_label()}.txt'

    def _load_offset_records(self, yaml_path):
        """加载偏移表; 文件缺失时告警并回退纯螺旋 (调用方继续，_offset_records 保持空)。"""
        import fast_solve
        path = self._offset_file_path(yaml_path)
        if not path.exists():
            rospy.logwarn(f"fast_solve 已开启但偏移文件不存在，回退纯螺旋搜索: {path}")
            return []
        records = fast_solve.load_offsets(path)
        rospy.loginfo(f"fast_solve: 加载 {len(records)} 条偏移记录 ({path.name})")
        return records

    # ------------------------------------------------------------------
    # 姿态计算 (两种模式通用)
    # ------------------------------------------------------------------

    @staticmethod
    def _quaternion_from_zxz(alpha_deg, beta_deg, gamma_deg):
        """Intrinsic Z-X-Z' (α → β → γ) 四元数。

        内旋 (绕体轴):
            1. 绕原始 Z 轴旋转 α
            2. 绕新的  X' 轴旋转 β
            3. 绕新的  Z''轴旋转 γ
        矩阵: R = Rz(α) * Rx(β) * Rz(γ)
        """
        q_z1 = quaternion_about_axis(math.radians(alpha_deg), (0, 0, 1))
        q_x  = quaternion_about_axis(math.radians(beta_deg),  (1, 0, 0))
        q_z2 = quaternion_about_axis(math.radians(gamma_deg), (0, 0, 1))
        q = quaternion_multiply(quaternion_multiply(q_z1, q_x), q_z2)
        return list(q)

    def _compute_ideal_alpha_beta(self, tx, ty, tz):
        """法兰盘 Z 轴指向 eye_position 的理想 (α, β)。

        法兰盘 Z 轴方向: [sin(α)sin(β), -cos(α)sin(β), cos(β)]
        最大化 法兰盘Z · (eye - target) 得 α = atan2(dx, -dy), β = atan2(√(dx²+dy²), dz)。
        """
        ex, ey, ez = self.eye_position
        dx, dy, dz = ex - tx, ey - ty, ez - tz
        alpha = math.degrees(math.atan2(dx, -dy))
        beta  = math.degrees(math.atan2(math.sqrt(dx * dx + dy * dy), dz))
        return alpha, beta

    def _compute_beta_for_alpha(self, tx, ty, tz, alpha_deg):
        """固定 α 下反算使法兰盘 Z 最接近 eye 方向的 β。

        β = atan2(sin(α)dx - cos(α)dy, dz)
        """
        ex, ey, ez = self.eye_position
        dx, dy, dz = ex - tx, ey - ty, ez - tz
        a = math.radians(alpha_deg)
        A = math.sin(a) * dx - math.cos(a) * dy
        return math.degrees(math.atan2(A, dz))

    # ------------------------------------------------------------------
    # 统一候选生成与搜索循环 (move_to 两种模式 / workspace_analyzer 共用)
    # ------------------------------------------------------------------

    def alpha_beta_candidates(self, x, y, z):
        """围绕几何理想 (α, β) 螺旋生成候选姿态 (搜索循环的唯一候选源)。

        返回 [(alpha, beta, d_alpha, d_beta), ...]:
        - best-first: 按偏离理想程度升序
        - 非理想 α 层的 β 搜索范围减半 (与原内联逻辑一致)
        - d_alpha/d_beta: 相对理想的偏移，供 fast_solve 记录/复用
        """
        ideal_alpha, ideal_beta = self._compute_ideal_alpha_beta(x, y, z)
        ideal_alpha = max(0.0, min(180.0, ideal_alpha))

        alpha_offsets = [0.0]
        for i in range(1, int(self.yaw_levels)):
            alpha_offsets.extend([i * self.yaw_step, -i * self.yaw_step])
        alpha_offsets.sort(key=abs)

        out = []
        for alpha_off in alpha_offsets:
            alpha = ideal_alpha + alpha_off
            if not (0.0 <= alpha <= 180.0):
                continue
            local_beta = self._compute_beta_for_alpha(x, y, z, alpha)
            eff_range = self.pitch_range if abs(alpha_off) < 1e-6 else self.pitch_range * 0.5
            n = int(eff_range / self.pitch_step)
            betas = [local_beta]
            for i in range(1, n + 1):
                betas.extend([local_beta + i * self.pitch_step,
                              local_beta - i * self.pitch_step])
            for beta in betas:
                if 0.0 <= beta <= 180.0:
                    out.append((alpha, beta,
                                alpha - ideal_alpha, beta - ideal_beta))
        return out

    def _priority_candidates(self, x, y, z):
        """搜索候选序列: fast_solve 修正锚螺旋在前, 理想锚螺旋兜底 (去重)。"""
        spiral = self.alpha_beta_candidates(x, y, z)   # 理想锚螺旋
        ideal_a = spiral[0][0] - spiral[0][2]          # 首候选 d=0 → 反解理想值
        ideal_b = spiral[0][1] - spiral[0][3]

        if not self._offset_records:
            return [(a, b, da, db, None) for a, b, da, db in spiral]

        import fast_solve
        off = fast_solve.weighted_offset(self._offset_records, x, y, z)
        if off is None:
            rospy.loginfo("  fast_solve: 无门限内邻居, 使用理想锚螺旋")
            return [(a, b, da, db, None) for a, b, da, db in spiral]
        da_bar, db_bar = off
        rospy.loginfo(f"  fast_solve: 加权偏移 (Δα={da_bar:+.1f}°, Δβ={db_bar:+.1f}°) 重锚螺旋")

        # 修正锚螺旋 = 理想锚螺旋整体平移 + 重裁剪 [0,180]
        # d 值相对几何理想值 (与分析器记录语义一致)
        reanchored = [
            (max(0.0, min(180.0, a + da_bar)),
             max(0.0, min(180.0, b + db_bar)),
             a + da_bar - ideal_a, b + db_bar - ideal_b, 'fast')
            for a, b, da, db in spiral]

        # 去重 (偏移极小/为 0 时两螺旋重合)
        seen, out = set(), []
        for cand in reanchored + [(a, b, da, db, None) for a, b, da, db in spiral]:
            key = (round(cand[0], 3), round(cand[1], 3))
            if key not in seen:
                seen.add(key)
                out.append(cand)
        return out

    @_synchronized
    def search_orientation(self, x, y, z, try_pose, abort_check=None):
        """统一 (α, β) 姿态搜索循环 — 服务模式/直连模式/analyzer 三方共用。

        仿真与实测一致性锚点: 候选顺序、裁剪、中止语义只存在这一份。

        :param try_pose: (alpha_deg, beta_deg) -> bool 后端:
             服务模式 = 调服务 + /end_pose 验证; 直连 = 规划+执行; analyzer = 仅规划
        :param abort_check: () -> bool，True 时中止 (analyzer 的 keep_running 标志)
        :return: (alpha, beta, d_alpha, d_beta, dist_or_None) 首个成功候选; 失败 None
        注意: try_pose 抛出的异常原样穿透 (ServiceCallTimeout 中止整个搜索依赖此契约)。
        失败如何报告由调用方决定 (move_to wrapper 打 ERROR; analyzer 静默防千点刷屏)。
        """
        for alpha, beta, da, db, dist in self._priority_candidates(x, y, z):
            if rospy.is_shutdown() or (abort_check is not None and abort_check()):
                return None
            if try_pose(alpha, beta):
                return (alpha, beta, da, db, dist)

        return None

    # ------------------------------------------------------------------
    # 规划 (仅直连模式)
    # ------------------------------------------------------------------

    def _reap_orphan_planners(self):
        """回收已自行结束的超时规划线程；仍在运行的保留跟踪。

        plan() 是阻塞调用，超时后无法取消，但每次规划前清理已死者，
        可将积累从"无限"压到"瞬时少数几个"。
        """
        if not self._orphan_threads:
            return
        alive = [t for t in self._orphan_threads if t.is_alive()]
        reaped = len(self._orphan_threads) - len(alive)
        self._orphan_threads = alive
        if reaped:
            rospy.loginfo(f"回收 {reaped} 个已结束的孤儿规划线程")
        if alive:
            rospy.logwarn(f"{len(alive)} 个规划线程仍未返回 (move_group 可能假死)")

    def _plan_with_timeout(self):
        """线程级硬超时规划。返回 (is_success, plan_result, elapsed_sec)。

        超时的 worker 线程被记入 _orphan_threads 跟踪，下次规划前回收。
        """
        self._reap_orphan_planners()

        holder = {"result": None, "exception": None}

        def worker():
            try:
                holder["result"] = self.move_group.plan()
            except Exception as e:
                holder["exception"] = e

        t = threading.Thread(target=worker, daemon=True)
        t0 = time.time()
        t.start()
        t.join(timeout=self.hard_timeout)
        elapsed = time.time() - t0

        if t.is_alive():
            self._orphan_threads.append(t)
            return False, None, elapsed
        if holder["exception"]:
            rospy.logwarn(f"规划异常: {holder['exception']}")
            return False, None, elapsed

        r = holder["result"]
        ok = r[0] if isinstance(r, tuple) else r
        return ok, r, elapsed

    def _execute(self, plan_result):
        """执行规划结果并清理。"""
        traj = plan_result[1] if isinstance(plan_result, tuple) else plan_result
        self.move_group.execute(traj, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    # ------------------------------------------------------------------
    # 障碍物管理 (两种模式通用 — PlanningSceneInterface 基于主题)
    # ------------------------------------------------------------------

    def _add_safety_walls(self):
        """根据 safety_bbox 在规划场景中添加六面安全墙壁。

        PlanningScene 不可用时抛 RuntimeError — safety_bbox 是硬约束，
        不允许在无碰撞约束下运动。
        """
        if self._ensure_scene() is None:
            raise RuntimeError(
                "PlanningScene 不可用，无法添加安全墙壁。"
                "请确保 move_group 已启动:\n"
                "  roslaunch piper_no_gripper_moveit demo.launch "
                f"can_port:={self.can_port or '<can_port>'}"
            )

        bbox = self.safety_bbox
        rospy.loginfo(f"添加安全墙壁: x={bbox.get('x', [-0.7, 0.7])}, y={bbox['y']}, z={bbox['z']}")

        self._remove_safety_walls()
        rospy.sleep(0.3)
        self._walls_added = True

        WALL_THICKNESS = 0.01
        x_min, x_max = bbox.get('x', (-0.7, 0.7))
        y_min, y_max = bbox['y']
        z_min, z_max = bbox['z']
        xr = x_max - x_min
        yr = y_max - y_min
        zr = z_max - z_min
        x_mid = x_min + xr / 2
        y_mid = y_min + yr / 2
        z_mid = z_min + zr / 2

        def _make_pose(x, y, z):
            p = PoseStamped()
            p.header.frame_id = self._frame_id
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = z
            return p

        self.scene.add_box(self._wall_names[0],
                           _make_pose(x_mid, y_mid, z_min - WALL_THICKNESS / 2),
                           size=(xr, yr, WALL_THICKNESS))
        self.scene.add_box(self._wall_names[1],
                           _make_pose(x_mid, y_mid, z_max + WALL_THICKNESS / 2),
                           size=(xr, yr, WALL_THICKNESS))
        self.scene.add_box(self._wall_names[2],
                           _make_pose(x_mid, y_max + WALL_THICKNESS / 2, z_mid),
                           size=(xr, WALL_THICKNESS, zr))
        self.scene.add_box(self._wall_names[3],
                           _make_pose(x_mid, y_min - WALL_THICKNESS / 2, z_mid),
                           size=(xr, WALL_THICKNESS, zr))
        x_pos = PoseStamped()
        x_pos.header.frame_id = self._frame_id
        x_pos.pose.position.x = x_max + WALL_THICKNESS / 2
        x_pos.pose.position.y = y_mid
        x_pos.pose.position.z = z_mid
        self.scene.add_box(self._wall_names[4], x_pos, size=(WALL_THICKNESS, yr, zr))
        x_neg = PoseStamped()
        x_neg.header.frame_id = self._frame_id
        x_neg.pose.position.x = x_min - WALL_THICKNESS / 2
        x_neg.pose.position.y = y_mid
        x_neg.pose.position.z = z_mid
        self.scene.add_box(self._wall_names[5], x_neg, size=(WALL_THICKNESS, yr, zr))

        rospy.sleep(0.7)
        rospy.loginfo("安全墙壁已添加。")

    def _remove_safety_walls(self):
        if self.scene is None or not self._walls_added:
            return
        for name in self._wall_names:
            self.scene.remove_world_object(name)
        self._walls_added = False

    def _add_obstacles(self):
        if not self.obstacles:
            return
        if self._ensure_scene() is None:
            raise RuntimeError(
                "PlanningScene 不可用，无法添加障碍物 (配置了 "
                f"{len(self.obstacles)} 个)。"
                "请确保 move_group 已启动:\n"
                "  roslaunch piper_no_gripper_moveit demo.launch "
                f"can_port:={self.can_port or '<can_port>'}"
            )
        rospy.loginfo(f"添加 {len(self.obstacles)} 个障碍物")
        for obs in self.obstacles:
            name = obs['name']
            x_min, x_max = obs['x']
            y_min, y_max = obs['y']
            z_min, z_max = obs['z']
            ps = PoseStamped()
            ps.header.frame_id = self._frame_id
            ps.pose.position.x = (x_min + x_max) / 2
            ps.pose.position.y = (y_min + y_max) / 2
            ps.pose.position.z = (z_min + z_max) / 2
            self.scene.add_box(name, ps, size=(x_max - x_min, y_max - y_min, z_max - z_min))
        rospy.sleep(0.3)
        rospy.loginfo("障碍物已添加。")

    def _remove_obstacles(self):
        # 基于场景实际状态清理而非当前配置列表:
        # 历史运行遗留的对象 (配置已改名/删除的障碍物) 也必须移除
        if self.scene is None:
            return
        try:
            known = self.scene.get_known_object_names()
        except Exception:
            return
        wall_set = set(self._wall_names)
        for name in known:
            if name and name not in wall_set:
                self.scene.remove_world_object(name)

    @_synchronized
    def clear_obstacles(self):
        """仅移除场景中的安全墙壁和障碍物，不关闭 MoveIt 连接。"""
        self._remove_obstacles()
        self._remove_safety_walls()
        rospy.loginfo("场景障碍物已清除。")

    def get_config_label(self):
        """返回当前障碍物配置的简短标签，用于文件命名等。"""
        if self.safety_bbox is None and not self.obstacles:
            return "no_obs"
        parts = ["obs"]
        bbox = self.safety_bbox or {}
        for axis in ('x', 'y', 'z'):
            lo, hi = bbox.get(axis, (None, None))
            if lo is not None:
                parts.append(f"{axis}{lo}_{hi}")
        if self.obstacles:
            parts.append(f"n{len(self.obstacles)}")
            for i, obs in enumerate(self.obstacles):
                if i <= 2:
                    parts.append(obs['name'])
        return '_'.join(parts)

    # ------------------------------------------------------------------
    # 公开接口: move_to (带 yaw/pitch 搜索)
    # ------------------------------------------------------------------

    @_synchronized
    def move_to(self, x, y, z):
        """
        移动到 (x, y, z)，法兰盘 Z 轴尽可能正对 eye_position。

        直连模式: 在 (yaw, pitch) 空间中螺旋搜索，roll=0。
        服务模式: 从理想姿态出发，依次尝试 pitch 候选直到末端位姿发生变化。
        返回: True / False
        """
        if self._use_service:
            return self._service_move_to(x, y, z)
        else:
            return self._inprocess_move_to(x, y, z)

    def _inprocess_move_to(self, x, y, z):
        """直连模式: 统一搜索循环 + 规划执行后端。"""
        ideal_alpha, ideal_beta = self._compute_ideal_alpha_beta(x, y, z)
        rospy.loginfo(f"move_to({x:.3f}, {y:.3f}, {z:.3f}) | "
                      f"ideal α={ideal_alpha:.1f}° β={ideal_beta:.1f}°")

        def _try(alpha, beta):
            q = self._quaternion_from_zxz(alpha, beta, 0.0)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

            self.move_group.set_pose_target(pose)
            ok, plan, elapsed = self._plan_with_timeout()
            if plan is None:
                rospy.loginfo(f"    [{elapsed * 1000:5.0f}ms] timeout")
                self.move_group.clear_pose_targets()
                return False
            if ok:
                rospy.loginfo(f"  >> SUCCESS [{elapsed * 1000:5.0f}ms] "
                              f"α={alpha:.1f}°(Δ{alpha - ideal_alpha:+.1f}°) "
                              f"β={beta:.1f}°(Δ{beta - ideal_beta:+.1f}°) γ=0.0°")
                self._execute(plan)
                return True
            rospy.loginfo(f"    [{elapsed * 1000:5.0f}ms] no solution")
            return False

        result = self.search_orientation(x, y, z, _try)
        if result is None:
            rospy.logerr(f"move_to 失败: ({x:.3f}, {y:.3f}, {z:.3f}) 在所有候选姿态下无解。")
        return result is not None

    def _service_move_to(self, x, y, z):
        """服务模式: 统一搜索循环 + 服务调用后端。

        服务端 go() 内部规划+执行合并。通过 rospy.set_param 驱动服务端
        planning_time 快速失败 (不可达姿态 ~0.05s 返回，臂不动)。
        可达到时 go() 阻塞至运动完成，通过 /end_pose 验证目标距离。
        """
        ns_prefix = f"{self.can_port}/" if self.can_port else ""
        rospy.set_param(f"{ns_prefix}joint_moveit_ctrl_server/planning_time",
                        self.planning_timeout)

        ideal_alpha, ideal_beta = self._compute_ideal_alpha_beta(x, y, z)
        rospy.loginfo(f"move_to({x:.3f}, {y:.3f}, {z:.3f}) [service] | "
                      f"ideal α={ideal_alpha:.1f}° β={ideal_beta:.1f}°")

        def _try(alpha, beta):
            q = self._quaternion_from_zxz(alpha, beta, 0.0)
            t0 = time.time()
            try:
                _call_joint_moveit_ctrl_endpose(x, y, z, q[0], q[1], q[2], q[3],
                                                max_velocity=0.3, max_acceleration=0.3,
                                                planning_time=self.planning_timeout,
                                                ns=self.can_port)
            except ServiceCallTimeout:
                raise                      # 系统级故障: 穿透循环，整体中止
            except rospy.ServiceException as e:
                rospy.logwarn(f"  服务调用异常: {e}")
                return False

            elapsed = time.time() - t0
            cur_pose = self._get_end_pose_from_topic()
            if cur_pose is None:
                return False

            dist = math.sqrt((cur_pose.position.x - x) ** 2 +
                             (cur_pose.position.y - y) ** 2 +
                             (cur_pose.position.z - z) ** 2)
            if dist < 0.02:
                rospy.loginfo(f"  >> SUCCESS [service] [{elapsed*1000:5.0f}ms] "
                              f"α={alpha:.1f}°(Δ{alpha - ideal_alpha:+.1f}°) "
                              f"β={beta:.1f}°(Δ{beta - ideal_beta:+.1f}°) "
                              f"γ=0.0° dist_to_target={dist:.3f}m")
                return True
            rospy.loginfo(f"    [{elapsed*1000:5.0f}ms] dist={dist:.3f}m, try next")
            return False

        result = self.search_orientation(x, y, z, _try)
        if result is None:
            rospy.logerr(f"move_to 失敗 [service]: ({x:.3f}, {y:.3f}, {z:.3f}) 无解。")
        return result is not None

    # ------------------------------------------------------------------
    # 公开接口: move_to_with_orientation
    # ------------------------------------------------------------------

    @_synchronized
    def move_to_with_orientation(self, x, y, z, alpha_deg, beta_deg, gamma_deg=0.0):
        """
        以指定 Z-X-Z' 姿态移动到 (x, y, z)，跳过搜索直接规划/执行。

        :param alpha_deg: 绕原始 Z 轴旋转角度 (度)
        :param beta_deg:  绕新的 X' 轴旋转角度 (度)
        :param gamma_deg: 绕新的 Z''轴旋转角度 (度)
        返回: True / False
        """
        q = self._quaternion_from_zxz(alpha_deg, beta_deg, gamma_deg)

        if self._use_service:
            prev_pose = self._get_end_pose_from_topic()
            prev_pos = (prev_pose.position.x, prev_pose.position.y, prev_pose.position.z) if prev_pose else None

            try:
                _call_joint_moveit_ctrl_endpose(x, y, z, q[0], q[1], q[2], q[3],
                                                 max_velocity=0.3, max_acceleration=0.3,
                                                 planning_time=self.planning_timeout,
                                                 ns=self.can_port)
            except rospy.ServiceException as e:
                rospy.logerr(f"move_to_with_orientation 服务调用失败: {e}")
                return False

            cur_pose = self._get_end_pose_from_topic()
            if cur_pose is not None and prev_pos is not None:
                cur_pos = (cur_pose.position.x, cur_pose.position.y, cur_pose.position.z)
                moved = math.sqrt(sum((a - b) ** 2 for a, b in zip(cur_pos, prev_pos)))
                if moved > 0.005:
                    rospy.loginfo(f"move_to_with_orientation [service] ({x:.3f}, {y:.3f}, {z:.3f}) "
                                  f"α={alpha_deg:.1f}° β={beta_deg:.1f}° γ={gamma_deg:.1f}° moved={moved:.3f}m")
                    return True
                else:
                    rospy.logwarn(f"move_to_with_orientation [service] 臂未移动 (delta={moved:.4f}m)")
                    return False

            rospy.loginfo(f"move_to_with_orientation [service] ({x:.3f}, {y:.3f}, {z:.3f}) "
                          f"α={alpha_deg:.1f}° β={beta_deg:.1f}° γ={gamma_deg:.1f}°")
            return True

        # 直连模式
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

        self.move_group.set_pose_target(pose)
        ok, plan, elapsed = self._plan_with_timeout()

        if plan is None:
            rospy.logwarn(f"move_to_with_orientation [{elapsed*1000:.0f}ms] timeout")
            self.move_group.clear_pose_targets()
            return False

        if ok:
            rospy.loginfo(f"move_to_with_orientation [{elapsed*1000:.0f}ms] success")
            self._execute(plan)
            return True
        else:
            rospy.logwarn(f"move_to_with_orientation [{elapsed*1000:.0f}ms] no solution")
            self.move_group.clear_pose_targets()
            return False

    # ------------------------------------------------------------------
    # 公开接口: move_to_joints
    # ------------------------------------------------------------------

    @_synchronized
    def move_to_joints(self, joint_values):
        """
        关节空间运动。joint_values: 6 元素列表 (rad)。
        返回: True / False
        """
        if self._use_service:
            prev_pose = self._get_end_pose_from_topic()
            prev_pos = (prev_pose.position.x, prev_pose.position.y, prev_pose.position.z) if prev_pose else None

            try:
                _call_joint_moveit_ctrl_arm(list(joint_values),
                                            max_velocity=0.3, max_acceleration=0.3,
                                            planning_time=self.planning_timeout,
                                            ns=self.can_port)
            except rospy.ServiceException as e:
                rospy.logerr(f"move_to_joints 服务调用失败: {e}")
                return False

            cur_pose = self._get_end_pose_from_topic()
            if cur_pose is not None and prev_pos is not None:
                cur_pos = (cur_pose.position.x, cur_pose.position.y, cur_pose.position.z)
                moved = math.sqrt(sum((a - b) ** 2 for a, b in zip(cur_pos, prev_pos)))
                rospy.loginfo(f"move_to_joints [service]: joints={[f'{j:.2f}' for j in joint_values]} "
                              f"moved={moved:.3f}m")
                return moved > 0.001
            return True

        # 直连模式
        self.move_group.set_joint_value_target(joint_values)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        return success

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------

    def _get_end_pose_from_topic(self):
        """从 end_pose 主题获取当前末端位姿 (仅服务模式使用)。"""
        topic = f"{self.can_port}/end_pose" if self.can_port else "end_pose"
        try:
            msg = rospy.wait_for_message(topic, PoseStamped, timeout=0.5)
            return msg.pose
        except rospy.ROSException:
            return None

    @_synchronized
    def get_current_pose(self):
        """返回当前末端位姿 (geometry_msgs/Pose)。"""
        if self._use_service:
            pose = self._get_end_pose_from_topic()
            if pose is not None:
                return pose
            rospy.logwarn("无法从 /end_pose 获取当前位姿")
            return Pose()

        return self.move_group.get_current_pose().pose

    @_synchronized
    def get_current_zxz(self):
        """返回当前末端 Z-X-Z' 欧拉角 (α_deg, β_deg, γ_deg)。"""
        pose = self.get_current_pose()
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        r02 = 2.0 * (qx * qz + qw * qy)
        r12 = 2.0 * (qy * qz - qw * qx)
        r20 = 2.0 * (qx * qz - qw * qy)
        r21 = 2.0 * (qy * qz + qw * qx)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        beta = math.acos(max(-1.0, min(1.0, r22)))
        sin_beta = math.sin(beta)

        if abs(sin_beta) > 1e-6:
            alpha = math.atan2(r02, -r12)
            gamma = math.atan2(r20, r21)
        else:
            gamma = 0.0
            r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
            r01 = 2.0 * (qx * qy - qw * qz)
            if beta < math.pi / 2.0:
                alpha = math.atan2(-r01, r00)
            else:
                alpha = math.atan2(r01, r00)

        return math.degrees(alpha), math.degrees(beta), math.degrees(gamma)

    @_synchronized
    def print_current_pose(self):
        """日志打印当前末端位姿 (Z-X-Z' 欧拉角)。"""
        pose = self.get_current_pose()
        alpha, beta, gamma = self.get_current_zxz()
        rospy.loginfo(f"[Pose] xyz=[{pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}] "
                      f"zxz=[α={alpha:.1f}° β={beta:.1f}° γ={gamma:.1f}°]")

    @_synchronized
    def shutdown(self):
        """关闭连接并清理场景障碍物。"""
        self._remove_obstacles()
        self._remove_safety_walls()
        if not self._use_service and _HAS_MOVEIT:
            moveit_commander.roscpp_shutdown()
        rospy.loginfo("PiperArmController 已关闭。")


# ------------------------------------------------------------------
# Demo (服务模式，用于真实机械臂控制)
# ------------------------------------------------------------------

if __name__ == "__main__":
    _name = sys.argv[1] if len(sys.argv) > 1 else "upper"

    rospy.init_node("piper_arm_controller_demo", anonymous=True)
    ctrl = PiperArmController.from_yaml(_name, use_service=True)

    # 回零
    ctrl.move_to_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rospy.sleep(0.5)
    ctrl.print_current_pose()

    # 法兰盘对眼位移动 (safety_bbox 内)
    # ctrl.move_to(0.3, 0.0, 0.2)
    rospy.sleep(1)
    ctrl.print_current_pose()

    ctrl.shutdown()
