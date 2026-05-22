#!/usr/bin/env python3
"""实时监听机械臂末端法兰盘位姿，按 g 打印当前位姿，按 ESC 退出。

同时发布法兰盘/末端工具坐标系可视化 (LINE_LIST, X=红 Y=绿 Z=蓝)。

用法:
    rosrun moveit_ctrl end_pose_monitor.py              # 全局命名空间
    rosrun moveit_ctrl end_pose_monitor.py piper_lower  # 指定 can_port
    rosrun moveit_ctrl end_pose_monitor.py lower        # 从 YAML 短名称读取 can_port

按键:
    g    — 打印当前法兰盘 / 工具位姿
    t    — 切换可视化: flange_only → tool_only → both → flange_only
    ESC  — 退出
"""

import math
import sys
import termios
import tty
import select
import threading
from pathlib import Path

import yaml
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker


_SCRIPT_DIR = Path(__file__).resolve().parent

AXIS_LENGTH = 0.16   # 坐标轴长度 (米)
AXIS_RADIUS = 0.002  # 线宽


# ------------------------------------------------------------------
# 工具位姿计算
# ------------------------------------------------------------------

def _quaternion_from_zxz(alpha_deg, beta_deg, gamma_deg):
    """Intrinsic Z-X-Z' Euler → quaternion [x, y, z, w]."""
    a = math.radians(alpha_deg)
    b = math.radians(beta_deg)
    g = math.radians(gamma_deg)

    c1, s1 = math.cos(a / 2), math.sin(a / 2)
    c2, s2 = math.cos(b / 2), math.sin(b / 2)
    c3, s3 = math.cos(g / 2), math.sin(g / 2)

    # q = q_z(α) * q_x(β) * q_z(γ)
    qw = c1 * c2 * c3 - s1 * c2 * s3
    qx = c1 * s2 * c3 + s1 * s2 * s3
    qy = s1 * s2 * c3 - c1 * s2 * s3
    qz = s1 * c2 * c3 + c1 * c2 * s3
    return [qx, qy, qz, qw]


def _quat_multiply(q1, q0):
    """Hamilton product q1 * q0 (apply q0 first, then q1)."""
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return [
        w1 * x0 + x1 * w0 + y1 * z0 - z1 * y0,
        w1 * y0 - x1 * z0 + y1 * w0 + z1 * x0,
        w1 * z0 + x1 * y0 - y1 * x0 + z1 * w0,
        w1 * w0 - x1 * x0 - y1 * y0 - z1 * z0,
    ]


def _quat_rotate(q, v):
    """用四元数 q 旋转向量 v。返回 (x, y, z)。"""
    qv = [v[0], v[1], v[2], 0.0]
    q_conj = [-q[0], -q[1], -q[2], q[3]]
    t = _quat_multiply(q, qv)
    r = _quat_multiply(t, q_conj)
    return (r[0], r[1], r[2])


def compute_tool_pose(flange_pose, tool_config):
    """从法兰盘位姿 + 工具偏置 (先旋转再平移) 计算工具位姿。

    tool_config = {'translation': [tx, ty, tz],   # 米, 在旋转后的工具坐标系中
                   'rotation_zxz': [α, β, γ]}     # 度, 内旋 Z-X-Z'

    变换链: T_tool = T_flange * T_offset
            T_offset = translate(t) * rotate(R_zxz)
    即: 先把法兰盘坐标系旋转 R_zxz，再沿旋转后的轴平移 t。

    返回 geometry_msgs/Pose。
    """
    t = tool_config['translation']
    rzxz = tool_config['rotation_zxz']

    q_offset = _quaternion_from_zxz(rzxz[0], rzxz[1], rzxz[2])

    q_flange = [flange_pose.orientation.x,
                flange_pose.orientation.y,
                flange_pose.orientation.z,
                flange_pose.orientation.w]

    # 工具在世界系中的位置: p_tool = p_flange + R_flange * (R_offset * t)
    t_rotated = _quat_rotate(q_offset, t)
    t_world = _quat_rotate(q_flange, t_rotated)

    tool = Pose()
    tool.position.x = flange_pose.position.x + t_world[0]
    tool.position.y = flange_pose.position.y + t_world[1]
    tool.position.z = flange_pose.position.z + t_world[2]

    q_tool = _quat_multiply(q_flange, q_offset)
    tool.orientation.x = q_tool[0]
    tool.orientation.y = q_tool[1]
    tool.orientation.z = q_tool[2]
    tool.orientation.w = q_tool[3]

    return tool


def pose_from_config(config):
    """从 {translation: [x,y,z], rotation_zxz: [α,β,γ]} 构造 Pose。"""
    p = Pose()
    p.position.x, p.position.y, p.position.z = config['translation']
    q = _quaternion_from_zxz(*config['rotation_zxz'])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


def compose_poses(base, offset):
    """位姿合成: T = T_base * T_offset。返回 Pose。"""
    q_base = [base.orientation.x, base.orientation.y,
              base.orientation.z, base.orientation.w]
    q_offset = [offset.orientation.x, offset.orientation.y,
                offset.orientation.z, offset.orientation.w]

    p_rot = _quat_rotate(q_base,
                         [offset.position.x, offset.position.y, offset.position.z])

    result = Pose()
    result.position.x = base.position.x + p_rot[0]
    result.position.y = base.position.y + p_rot[1]
    result.position.z = base.position.z + p_rot[2]

    q_result = _quat_multiply(q_base, q_offset)
    result.orientation.x = q_result[0]
    result.orientation.y = q_result[1]
    result.orientation.z = q_result[2]
    result.orientation.w = q_result[3]
    return result


# ------------------------------------------------------------------
# Z-X-Z' extraction
# ------------------------------------------------------------------

def _zxz_from_quaternion(qx, qy, qz, qw):
    """四元数 → Intrinsic Z-X-Z' 欧拉角 (度)."""
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


# ------------------------------------------------------------------
# YAML helpers
# ------------------------------------------------------------------

def _resolve_can_port(arg):
    """短名称 → piper_<name>.yaml → can_port，否则直接返回 arg。"""
    path = Path(arg)
    if path.suffix not in ('.yaml', '.yml'):
        path = _SCRIPT_DIR / 'cfg' / f'piper_{arg}.yaml'
    if path.exists():
        cfg = yaml.safe_load(path.read_text())
        return cfg.get('arm', {}).get('can_port', arg)
    return arg


def _load_tool_config(arg):
    """从 YAML 加载 tool 配置。arg 为空返回 None。"""
    if not arg:
        return None
    path = Path(arg)
    if path.suffix not in ('.yaml', '.yml'):
        path = _SCRIPT_DIR / 'cfg' / f'piper_{arg}.yaml'
    if path.exists():
        cfg = yaml.safe_load(path.read_text())
        return cfg.get('tool')
    return None


# ------------------------------------------------------------------
# ARROW axis publishing helpers
# ------------------------------------------------------------------

# 把 ARROW 的默认 +X 方向转到 +Y / +Z 的四元数
_Q_ALIGN_Y = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]   # +90° about Z
_Q_ALIGN_Z = [0.0, math.sin(-math.pi / 4), 0.0, math.cos(-math.pi / 4)]  # -90° about Y


def _publish_axes(pubs, pose, length, frame_id, stamp):
    """发布三轴 ARROW marker (X=红, Y=绿, Z=蓝)。

    ARROW 默认沿 +X 绘制，所以:
      X marker → 直接用当前姿态
      Y marker → 姿态 * _Q_ALIGN_Y  (把 +X 转到 +Y)
      Z marker → 姿态 * _Q_ALIGN_Z  (把 +X 转到 +Z)
    """
    q = pose.orientation
    q_pose = [q.x, q.y, q.z, q.w]

    # 各轴所需的姿态
    qx = q_pose                        # +X 方向
    qy = _quat_multiply(q_pose, _Q_ALIGN_Y)   # +Y 方向
    qz = _quat_multiply(q_pose, _Q_ALIGN_Z)   # +Z 方向

    axes = [
        ('x', 0, qx, (1.0, 0.2, 0.2)),
        ('y', 1, qy, (0.2, 1.0, 0.2)),
        ('z', 2, qz, (0.2, 0.2, 1.0)),
    ]

    for name, mid, q_axis, color in axes:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = name
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position = pose.position
        m.pose.orientation = Quaternion(q_axis[0], q_axis[1], q_axis[2], q_axis[3])
        m.scale.x = length      # 箭身长
        m.scale.y = length * 0.05  # 箭头宽
        m.scale.z = length * 0.05  # 箭头高
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.8
        pubs[name].publish(m)


# ------------------------------------------------------------------
# Keyboard listener
# ------------------------------------------------------------------

class KeyboardListener:
    """非阻塞键盘监听。"""

    def __init__(self):
        self._char = None
        self._lock = threading.Lock()
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def poll(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            with self._lock:
                self._char = ch
            return ch
        return None

    def restore(self):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)


# ------------------------------------------------------------------
# Monitor
# ------------------------------------------------------------------

class EndPoseMonitor:
    """监听 end_pose，发布法兰/工具坐标系，按键交互。"""

    def __init__(self, can_port="", tool_config=None, arm_in_ccs_config=None):
        self._can_port = can_port
        self._tool_config = tool_config
        self._arm_in_ccs_config = arm_in_ccs_config
        self._pose = None
        self._frame_id = None
        self._lock = threading.Lock()

        # 可视化模式: 0=flange only, 1=tool only, 2=both
        self._vis_mode = 0 if tool_config else 0
        self._has_tool = tool_config is not None
        self._has_ccs = arm_in_ccs_config is not None

        topic = f"{can_port}/end_pose" if can_port else "end_pose"
        self._sub = rospy.Subscriber(topic, PoseStamped, self._callback)

        ns = f"{can_port}/" if can_port else ""

        # 法兰盘坐标系 publisher
        self._pubs_flange = {
            'x': rospy.Publisher(f'{ns}flange_frame_x', Marker, queue_size=1),
            'y': rospy.Publisher(f'{ns}flange_frame_y', Marker, queue_size=1),
            'z': rospy.Publisher(f'{ns}flange_frame_z', Marker, queue_size=1),
        }

        # 工具坐标系 publisher (仅当 tool_config 存在)
        self._pubs_tool = {}
        if self._has_tool:
            self._pubs_tool = {
                'x': rospy.Publisher(f'{ns}tool_frame_x', Marker, queue_size=1),
                'y': rospy.Publisher(f'{ns}tool_frame_y', Marker, queue_size=1),
                'z': rospy.Publisher(f'{ns}tool_frame_z', Marker, queue_size=1),
            }

        rospy.loginfo(f"EndPoseMonitor: listening on /{topic}")
        if self._has_tool:
            rospy.loginfo(f"Tool config: t={tool_config['translation']}m, "
                          f"rot_zxz={tool_config['rotation_zxz']}deg")
        if self._has_ccs:
            rospy.loginfo(f"Arm-in-CCS: t={arm_in_ccs_config['translation']}m, "
                          f"rot_zxz={arm_in_ccs_config['rotation_zxz']}deg")
        rospy.loginfo("Keys: g=print  t=toggle view  ESC=exit")

    def _callback(self, msg):
        with self._lock:
            self._pose = msg.pose
            self._frame_id = msg.header.frame_id

    @property
    def pose(self):
        with self._lock:
            return self._pose

    @property
    def frame_id(self):
        with self._lock:
            return self._frame_id or "dummy_link"

    @property
    def tool_pose(self):
        """根据法兰盘位姿 + 工具偏置计算工具位姿。"""
        flange = self.pose
        if flange is None or self._tool_config is None:
            return None
        return compute_tool_pose(flange, self._tool_config)

    def toggle_view(self):
        """切换可视化模式。"""
        if not self._has_tool:
            return
        self._vis_mode = (self._vis_mode + 1) % 3
        labels = {0: "flange only", 1: "tool only", 2: "flange + tool"}
        rospy.loginfo(f"View mode: {labels[self._vis_mode]}")

    def publish(self):
        """根据当前模式发布坐标系 marker。"""
        pose = self.pose
        if pose is None:
            return

        frame_id = self.frame_id
        now = rospy.Time.now()

        show_flange = self._vis_mode in (0, 2)
        show_tool = self._vis_mode in (1, 2) and self._has_tool

        if show_flange:
            _publish_axes(self._pubs_flange, pose, AXIS_LENGTH, frame_id, now)

        if show_tool:
            tool = self.tool_pose
            if tool is not None:
                _publish_axes(self._pubs_tool, tool, AXIS_LENGTH, frame_id, now)

    @staticmethod
    def pose_to_str(pose, label=""):
        if pose is None:
            return "no data yet"
        pos = pose.position
        ori = pose.orientation
        alpha, beta, gamma = _zxz_from_quaternion(ori.x, ori.y, ori.z, ori.w)
        prefix = f"[{label}] " if label else ""
        return (
            f"\n{prefix}Position (xyz):     [{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}]  "
            f"\n{prefix}Orientation (wxyz): [{ori.w:.4f}, {ori.x:.4f}, {ori.y:.4f}, {ori.z:.4f}]  "
            f"\n{prefix}Euler Z-X-Z' (α-β-γ): [{alpha:.1f}°, {beta:.1f}°, {gamma:.1f}°]\n"
        )

    def print(self):
        rospy.loginfo(self.pose_to_str(self.pose, "flange"))
        if self._has_tool:
            tool = self.tool_pose
            if tool is not None:
                rospy.loginfo(self.pose_to_str(tool, "tool"))
                if self._has_ccs:
                    arm_in_ccs = pose_from_config(self._arm_in_ccs_config)
                    tool_in_ccs = compose_poses(arm_in_ccs, tool)
                    rospy.loginfo(self.pose_to_str(tool_in_ccs, "tool in CCS"))


# ------------------------------------------------------------------
# main
# ------------------------------------------------------------------

def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    can_port = _resolve_can_port(arg) if arg else ""
    tool_config = _load_tool_config(arg) if arg else None

    # 加载 arm_in_ccs 配置
    arm_in_ccs_config = None
    if arg:
        path = Path(arg)
        if path.suffix not in ('.yaml', '.yml'):
            path = _SCRIPT_DIR / 'cfg' / f'piper_{arg}.yaml'
        if path.exists():
            cfg = yaml.safe_load(path.read_text())
            arm_in_ccs_config = cfg.get('arm_in_ccs')

    rospy.init_node("end_pose_monitor", anonymous=True)

    monitor = EndPoseMonitor(can_port, tool_config, arm_in_ccs_config)
    kb = KeyboardListener()
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            ch = kb.poll()
            if ch:
                if ch == 'g':
                    monitor.print()
                elif ch == 't':
                    monitor.toggle_view()
                elif ord(ch) == 27:
                    rospy.loginfo("EndPoseMonitor: exit")
                    break
            monitor.publish()
            rate.sleep()
    finally:
        kb.restore()


if __name__ == '__main__':
    main()
