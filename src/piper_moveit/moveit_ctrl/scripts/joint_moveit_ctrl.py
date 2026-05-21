#!/usr/bin/env python3

import rospy
import time
import random
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest # pyright: ignore[reportAttributeAccessIssue]
from tf.transformations import quaternion_from_euler # type: ignore

def call_joint_moveit_ctrl_arm(joint_states, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_arm")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_arm", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_states = joint_states
        request.gripper = 0.0
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_arm")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_arm, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

def call_joint_moveit_ctrl_gripper(gripper_position, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_gripper")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_gripper", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_states = [0.0] * 6
        request.gripper = gripper_position
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_gripper")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_gripper, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

def call_joint_moveit_ctrl_piper(joint_states, gripper_position, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_piper")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_piper", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_states = joint_states
        request.gripper = gripper_position
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_piper")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_piper, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

def convert_endpose(endpose):
    if len(endpose) == 6:
        x, y, z, roll, pitch, yaw = endpose
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        return [x, y, z, qx, qy, qz, qw]

    elif len(endpose) == 7:
        return endpose  # 直接返回四元数

    else:
        raise ValueError("Invalid endpose format! Must be 6 (Euler) or 7 (Quaternion) values.")

def call_joint_moveit_ctrl_endpose(endpose, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_endpose")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_endpose", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        
        request.joint_states = [0.0] * 6  # 填充6个关节状态
        request.gripper = 0.0
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration
        request.joint_endpose = convert_endpose(endpose)  # 自动转换

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_endpose")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_endpose, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

# 此处关节限制仅为测试使用，实际关节限制以READEME中为准
def randomval():
    arm_position = [
        random.uniform(-0.2, 0.2),  # 关节1
        random.uniform(0, 0.5),  # 关节2
        random.uniform(-0.5, 0),  # 关节3
        random.uniform(-0.2, 0.2),  # 关节4
        random.uniform(-0.2, 0.2),  # 关节5
        random.uniform(-0.2, 0.2)   # 关节6
    ]
    gripper_position = random.uniform(0, 0.035)

    return arm_position, gripper_position

import moveit_commander # type: ignore

def move_robot():
    # 初始化 MoveIt! 相关组件
    moveit_commander.roscpp_initialize([])
    move_group = moveit_commander.MoveGroupCommander("arm")  # pyright: ignore[reportCallIssue] # 可以根据需要修改为 "arm"、"gripper"或"piper"

    # 获取当前关节值
    # joint_goal = move_group.get_current_joint_values()    
    joint_goal = [
        0.0,           # joint1: [-2.618, 2.618]
        0.0,          # joint2: [0, 3.14] → 中间值 90° = 1.57 rad
        0.0,       # joint3: [-2.967, 0] → 中间值 -85° = -1.4835 rad
        0.0,           # joint4: [-1.745, 1.745]
        0.0,           # joint5: [-1.22, 1.22]
        0.0            # joint6: [-2.0944, 2.0944]
    ]

    # 设置并执行目标
    move_group.set_joint_value_target(joint_goal)
    success = move_group.go(wait=True)

    joint_goal = [
        0.0,           # joint1: [-2.618, 2.618]
        1.57,          # joint2: [0, 3.14] → 中间值 90° = 1.57 rad
        -1.4835,       # joint3: [-2.967, 0] → 中间值 -85° = -1.4835 rad
        0.0,           # joint4: [-1.745, 1.745]
        0.0,           # joint5: [-1.22, 1.22]
        0.0            # joint6: [-2.0944, 2.0944]
    ]

    # 设置并执行目标
    move_group.set_joint_value_target(joint_goal)
    success = move_group.go(wait=True)

    rospy.loginfo(f"Movement success: {success}")
    rospy.loginfo(f"joint_value: {move_group.get_current_joint_values()}")

    moveit_commander.roscpp_shutdown()

def main():
    rospy.init_node("test_joint_moveit_ctrl", anonymous=True)
    arm_position, gripper_position = [], 0
    for i in range(10): 
        # arm_position, _ = randomval()  # 机械臂控制
        # call_joint_moveit_ctrl_arm(arm_position, max_velocity=0.5, max_acceleration=0.5)
        # time.sleep(1)
        # _, gripper_position = randomval()  # 夹爪控制
        # call_joint_moveit_ctrl_gripper(gripper_position)
        # time.sleep(1)
        # arm_position, gripper_position = randomval()
        # call_joint_moveit_ctrl_piper(arm_position, gripper_position)  # 机械臂夹爪联合控制
        # time.sleep(1)
        # endpose_euler = [0.531014, -0.133376, 0.418909, -0.6052452780065936, 1.2265301318390152, -0.9107128036906411]
        # call_joint_moveit_ctrl_endpose(endpose_euler)  # 末端位置控制(欧拉角)
        # time.sleep(1)
        # endpose_quaternion = [0.531014, -0.133376, 0.418909, 0.02272779901175584, 0.6005891177332143, -0.18925185045722595, 0.7765049233012219]
        # call_joint_moveit_ctrl_endpose(endpose_quaternion)  # 末端位置控制(四元数)
        # time.sleep(1)
        arm_position = [0, 0, 0, 0, 0, 0]
        call_joint_moveit_ctrl_arm(arm_position, max_velocity=0.5, max_acceleration=0.5) # 回零
        time.sleep(1)

from geometry_msgs.msg import Pose, PoseStamped # type: ignore
import sys
import threading
from tf.transformations import euler_from_quaternion # type: ignore
import math
import rospy

# 人眼在机械臂基座坐标系中的位置 (超参数，每台机械臂需单独配置)
# 法兰盘运动到目标点时 Z 轴会尽可能指向此位置
EYE_POSITION = (1.0, 0.0, 0.6)  # (x, y, z) 单位: 米，典型值 [1.0, 0.0, 0.0] 或 [1.0, 0.0, 0.5]

class SafetyZoneManager:
    """
    管理 MoveIt 规划场景中的安全边界墙。
    """
    def __init__(self, scene, frame_id="dummy_link"):
        """
        :param scene: moveit_commander.PlanningSceneInterface() 的实例
        :param frame_id: 机械臂的基座坐标系
        """
        self.scene = scene
        self.frame_id = frame_id
        self.wall_names = [
            "safety_wall_floor",
            "safety_wall_ceiling",
            "safety_wall_y_pos",
            "safety_wall_y_neg"
        ]

    def add_safety_walls(self, bbox):
        """
        根据 bbox 定义，在场景中添加四个无限延伸的墙壁。
        """
        rospy.loginfo("Adding safety walls to the planning scene...")
        
        # 移除旧的墙壁，以防重复添加
        self.remove_safety_walls()
        rospy.sleep(0.5) # 等待场景更新

        # 定义墙壁的参数
        WALL_THICKNESS = 0.01  # 墙壁厚度 (1cm)
        WALL_LENGTH_X = 3.0    # 在X方向上设置一个很长的值来模拟无限

        y_min, y_max = bbox['y']
        z_min, z_max = bbox['z']
        
        y_range = y_max - y_min
        z_range = z_max - z_min
        
        # 1. 添加地板 (z = z_min)
        floor_pose = PoseStamped()
        floor_pose.header.frame_id = self.frame_id
        floor_pose.pose.position.x = 0 # 中心在X=0
        floor_pose.pose.position.y = y_min + y_range / 2
        floor_pose.pose.position.z = z_min - WALL_THICKNESS / 2
        self.scene.add_box(self.wall_names[0], floor_pose, size=(WALL_LENGTH_X, y_range, WALL_THICKNESS))

        # 2. 添加天花板 (z = z_max)
        ceiling_pose = PoseStamped()
        ceiling_pose.header.frame_id = self.frame_id
        ceiling_pose.pose.position.x = 0
        ceiling_pose.pose.position.y = y_min + y_range / 2
        ceiling_pose.pose.position.z = z_max + WALL_THICKNESS / 2
        self.scene.add_box(self.wall_names[1], ceiling_pose, size=(WALL_LENGTH_X, y_range, WALL_THICKNESS))

        # 3. 添加 Y 正方向的墙 (y = y_max)
        y_pos_wall_pose = PoseStamped()
        y_pos_wall_pose.header.frame_id = self.frame_id
        y_pos_wall_pose.pose.position.x = 0
        y_pos_wall_pose.pose.position.y = y_max + WALL_THICKNESS / 2
        y_pos_wall_pose.pose.position.z = z_min + z_range / 2
        self.scene.add_box(self.wall_names[2], y_pos_wall_pose, size=(WALL_LENGTH_X, WALL_THICKNESS, z_range))

        # 4. 添加 Y 负方向的墙 (y = y_min)
        y_neg_wall_pose = PoseStamped()
        y_neg_wall_pose.header.frame_id = self.frame_id
        y_neg_wall_pose.pose.position.x = 0
        y_neg_wall_pose.pose.position.y = y_min - WALL_THICKNESS / 2
        y_neg_wall_pose.pose.position.z = z_min + z_range / 2
        self.scene.add_box(self.wall_names[3], y_neg_wall_pose, size=(WALL_LENGTH_X, WALL_THICKNESS, z_range))

        rospy.sleep(1.0) # 关键：等待场景完全更新后再进行规划
        rospy.loginfo("Safety walls added. You can see them in RViz under 'Scene Objects'.")

    def remove_safety_walls(self):
        """
        从场景中移除所有安全墙壁。
        """
        for wall_name in self.wall_names:
            self.scene.remove_world_object(wall_name)
        rospy.loginfo("Safety walls removed.")

def print_current_pose(move_group):
    """
    打印当前位姿，使用符合直觉的 Yaw-Pitch-Roll (Intrinsic Z-Y-X) 顺序显示。
    """
    # 1. 获取当前位姿
    pose = move_group.get_current_pose().pose
    q_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    # 2. 转换为欧拉角
    # axes='rzyx' 代表 Intrinsic Z -> Y -> X
    # 返回的元组顺序对应 axes 的顺序，即 (angle_z, angle_y, angle_x) -> (yaw, pitch, roll)
    (yaw_rad, pitch_rad, roll_rad) = euler_from_quaternion(q_list, axes='rzyx')

    # 3. 转换为角度
    yaw_deg = math.degrees(yaw_rad)
    pitch_deg = math.degrees(pitch_rad)
    roll_deg = math.degrees(roll_rad)

    # 4. 打印 (格式优化，便于一眼看清)
    rospy.loginfo(
        f"[Current Pose Status]\n"
        f"  Position (XYZ):  [{pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}]\n"
        f"  Orientation (YPR):"
        f" Pitch: {pitch_deg:6.1f} deg "
        f"Yaw: {yaw_deg:6.1f} deg "
        f"Roll: {roll_deg:6.1f} deg"
    )

from tf.transformations import quaternion_about_axis, quaternion_multiply # type: ignore

def get_hybrid_quaternion(yaw_deg, pitch_deg, roll_deg):
    """
    Pan-Tilt-Twist (Intrinsic Z-Y-X) 四元数生成器
    """
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    # 1. Yaw (Z) - Global
    q_yaw = quaternion_about_axis(yaw, (0, 0, 1))
    # 2. Pitch (Y) - Intrinsic (Relative to Yaw)
    q_pitch = quaternion_about_axis(pitch, (0, 1, 0))
    # 3. Roll (X) - Intrinsic (Relative to Pitch)
    q_roll = quaternion_about_axis(roll, (1, 0, 0))

    # Order: Yaw -> Pitch -> Roll
    q_aim = quaternion_multiply(q_yaw, q_pitch)
    q_final = quaternion_multiply(q_aim, q_roll)
    return list(q_final)

def compute_ideal_yaw_pitch(target_x, target_y, target_z):
    """
    从 target 指向 EYE_POSITION 的方向向量，反算理想 (yaw, pitch)。
    yaw 为方向向量的水平方位角，pitch 为与垂直轴的夹角。
    返回: (yaw_deg, pitch_deg)
    """
    ex, ey, ez = EYE_POSITION
    dx = ex - target_x
    dy = ey - target_y
    dz = ez - target_z
    yaw_rad = math.atan2(dy, dx)
    pitch_rad = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return math.degrees(yaw_rad), math.degrees(pitch_rad)


def compute_pitch_to_eye(target_x, target_y, target_z, yaw_deg):
    """
    计算使法兰盘 Z 轴尽可能指向 EYE_POSITION 的 pitch 角度。
    在固定 yaw 的前提下，将眼位方向向量投影到法兰盘 Z 轴所在的垂直平面，反算 pitch。
    """
    ex, ey, ez = EYE_POSITION
    dx = ex - target_x
    dy = ey - target_y
    dz = ez - target_z

    yaw_rad = math.radians(yaw_deg)
    proj_xy = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)

    pitch_rad = math.atan2(proj_xy, dz)
    return math.degrees(pitch_rad)


def move_to_xyz(move_group, x, y, z, yaw_deg, pitch_deg, roll_deg):
    q = get_hybrid_quaternion(yaw_deg, pitch_deg, roll_deg)
    
    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    move_group.clear_pose_targets()
    move_group.set_pose_target(target_pose)
    
    # 4. 规划路径 (只计算，不移动)
    plan_result = move_group.plan()
    
    # 兼容 MoveIt 不同版本的返回值处理
    is_success = plan_result[0] if isinstance(plan_result, tuple) else plan_result

    # 5. 检查是否可达
    if is_success:
        rospy.loginfo(f"规划成功: 目标点 ({x}, {y}, {z}) 可达。正在执行运动...")
        
        # 6. 执行运动
        # 如果是 tuple (MoveIt 1.0), 取第二个元素作为 trajectory
        traj = plan_result[1] if isinstance(plan_result, tuple) else plan_result
        move_group.execute(traj, wait=True)
        
        # 运动后停止，清除目标
        move_group.stop()
        move_group.clear_pose_targets()
        move_group.clear_path_constraints()
        return True
    else:
        rospy.logwarn(f"规划失败: 目标点 ({x}, {y}, {z}) 不可达 (可能超出工作空间或碰撞)。")
        return False

def move_to_xyz_smart(move_group, x, y, z):
    """
    智能 XYZ 移动函数：
    输入: x, y, z
    逻辑: 自动计算符合 Pan-Tilt-Twist 限制的姿态
          1. Yaw: 自动对准目标方向 (atan2(y,x))，限制在 [-90, 90]
          2. Pitch: 优先尝试 90(垂直)，其次 60, 45, 0 (水平)
          3. Roll: 默认 0
    输出: Boolean (是否成功)
    """
    
    # --- 1. 计算 Yaw (自动对准目标) ---
    # 使用 atan2 计算目标在基座坐标系下的角度
    auto_yaw_rad = math.atan2(y, x)
    auto_yaw_deg = math.degrees(auto_yaw_rad)

    # 强制 Yaw 限制 [-90, 90]
    if not (-90.0 <= auto_yaw_deg <= 90.0):
        rospy.logwarn(f"SmartMove 失败: 目标 ({x}, {y}) 需要 Yaw={auto_yaw_deg:.1f}°，"
                      f"但这超出了 [-90, 90] 的限制。")
        return False

    # --- 2. 定义 Pitch 的搜索策略 ---
    # 优先级：垂直向下(90) -> 前下方(60, 45) -> 水平向前(0) -> 稍向后(120)
    # 这种顺序保证了机械臂总是倾向于以最自然的姿态抓取
    pitch_candidates = [90.0, 60.0, 45.0, 30.0, 0.0, 100.0]

    # --- 3. 定义 Roll 的搜索策略 ---
    # 通常 Roll=0 够用了，但为了防死锁，可以加一个 90
    roll_candidates = [0.0]

    rospy.loginfo(f"SmartMove 开始规划 -> 目标: xyz({x:.2f}, {y:.2f}, {z:.2f}) | "
                  f"自动 Yaw: {auto_yaw_deg:.1f}°")

    # --- 4. 迭代搜索可行解 ---
    for pitch in pitch_candidates:
        for roll in roll_candidates:
            # 构造四元数
            q = get_hybrid_quaternion(auto_yaw_deg, pitch, roll)
            
            # 设置目标
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]

            move_group.set_pose_target(target_pose)
            
            # 尝试规划 (plan only)
            plan_result = move_group.plan()
            is_success = plan_result[0] if isinstance(plan_result, tuple) else plan_result

            if is_success:
                rospy.loginfo(f">> 找到可行解! 使用姿态: Pitch={pitch}°, Roll={roll}°")
                
                # 执行运动
                traj = plan_result[1] if isinstance(plan_result, tuple) else plan_result
                move_group.execute(traj, wait=True)
                move_group.stop()
                move_group.clear_pose_targets()
                return True
            
            # 如果不成功，循环继续，尝试下一个 Pitch 角度...

    # 如果所有角度都试过了还是不行
    rospy.logerr(f"SmartMove 彻底失败: 目标点 ({x}, {y}, {z}) 在当前 Yaw={auto_yaw_deg:.1f}° 下，"
                 f"尝试了所有 Pitch 候选 {pitch_candidates} 均不可达。")
    return False

def _plan_worker(move_group, result_holder):
    try:
        result_holder["result"] = move_group.plan()
    except Exception as e:
        result_holder["exception"] = e


def plan_with_timeout(move_group, hard_timeout=0.5):
    """
    带硬超时的 MoveIt 规划。
    move_group 需预先 set_planning_time() 作为软超时，
    hard_timeout 为线程级硬超时兜底。
    返回: (is_success, plan_result, elapsed_sec)
    """
    result_holder = {"result": None, "exception": None}

    thread = threading.Thread(
        target=_plan_worker, args=(move_group, result_holder), daemon=True
    )
    t0 = time.time()
    thread.start()
    thread.join(timeout=hard_timeout)
    elapsed = time.time() - t0

    if thread.is_alive():
        return False, None, elapsed

    if result_holder["exception"]:
        rospy.logwarn(f"规划异常: {result_holder['exception']}")
        return False, None, elapsed

    plan_result = result_holder["result"]
    is_success = plan_result[0] if isinstance(plan_result, tuple) else plan_result
    return is_success, plan_result, elapsed


def move_to_xyz_face_eye(move_group, x, y, z,
                          yaw_step=10.0, yaw_levels=3,
                          pitch_step=5.0, pitch_range=20.0):
    """
    移动到目标位置 (x, y, z)，同时法兰盘正对 EYE_POSITION。
    在 (yaw, pitch) 空间中从理想值向外螺旋搜索:
      - yaw 层次: 理想 → ±step → ±2*step → ...
      - 每层 yaw 下重算局部理想 pitch，并在附近搜索
      - 非理想 yaw 时 pitch 搜索范围减半 (yaw 已妥协，不再叠加极端 pitch)
    Roll 固定为 0。
    """
    ideal_yaw, ideal_pitch = compute_ideal_yaw_pitch(x, y, z)
    ideal_yaw = max(-90.0, min(90.0, ideal_yaw))

    # 极速配置: 0.05s 算不出来大概率就是无解
    move_group.set_planning_time(0.05)
    move_group.set_num_planning_attempts(1)

    yaw_offsets = [0.0]
    for i in range(1, yaw_levels):
        yaw_offsets.extend([i * yaw_step, -i * yaw_step])
    yaw_offsets.sort(key=abs)

    rospy.loginfo(f"FaceEye 联合搜索 -> xyz({x:.2f}, {y:.2f}, {z:.2f}) | "
                  f"理想 (yaw={ideal_yaw:.1f}°, pitch={ideal_pitch:.1f}°) | "
                  f"yaw 层数 {yaw_levels} 步长 {yaw_step}° | pitch 步长 {pitch_step}°")

    for yaw_off in yaw_offsets:
        yaw = ideal_yaw + yaw_off
        if not (-90.0 <= yaw <= 90.0):
            continue

        local_pitch = compute_pitch_to_eye(x, y, z, yaw)
        effective_range = pitch_range if abs(yaw_off) < 1e-6 else pitch_range * 0.5
        num_steps = int(effective_range / pitch_step)

        pitch_candidates = [local_pitch]
        for i in range(1, num_steps + 1):
            pitch_candidates.append(local_pitch + i * pitch_step)
            pitch_candidates.append(local_pitch - i * pitch_step)
        pitch_candidates = [p for p in pitch_candidates if -120.0 <= p <= 120.0]

        if abs(yaw_off) < 1e-6:
            rospy.loginfo(f"  搜索 yaw={yaw:.1f}° (理想) 局部pitch={local_pitch:.1f}° "
                          f"范围 ±{effective_range:.0f}° [{len(pitch_candidates)} 个候选]")
        else:
            rospy.loginfo(f"  搜索 yaw={yaw:.1f}° (偏离 {yaw_off:+.0f}°) "
                          f"局部pitch={local_pitch:.1f}° 范围 ±{effective_range:.0f}° [{len(pitch_candidates)} 个候选]")

        for pitch in pitch_candidates:
            if rospy.is_shutdown():
                return False

            q = get_hybrid_quaternion(yaw, pitch, 0.0)

            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]

            move_group.set_pose_target(target_pose)
            is_success, plan_result, elapsed = plan_with_timeout(move_group)

            if not plan_result:
                rospy.loginfo(f"    [{elapsed*1000:5.0f}ms] (yaw={yaw:.0f}° pitch={pitch:.0f}°) 超时/异常，跳过")
                move_group.clear_pose_targets()
                continue

            if is_success:
                yaw_dev = yaw - ideal_yaw
                pitch_dev = pitch - ideal_pitch
                rospy.loginfo(f">> FaceEye 成功 [{elapsed*1000:5.0f}ms]: "
                              f"yaw={yaw:.1f}° (Δ{yaw_dev:+.1f}°) "
                              f"pitch={pitch:.1f}° (Δ{pitch_dev:+.1f}°)")
                traj = plan_result[1] if isinstance(plan_result, tuple) else plan_result
                move_group.execute(traj, wait=True)
                move_group.stop()
                move_group.clear_pose_targets()
                return True
            else:
                rospy.loginfo(f"    [{elapsed*1000:5.0f}ms] (yaw={yaw:.0f}° pitch={pitch:.0f}°) 无解")

    rospy.logerr(f"FaceEye 失败: ({x:.2f}, {y:.2f}, {z:.2f}) "
                 f"在 {yaw_levels} 层 yaw + {pitch_step}° 步长 pitch 搜索内无解。")
    return False


def move_robot_cartesian_demo():
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("test_xyz_moveit_with_safety_zone", anonymous=True)
    
    # 实例化 MoveIt 组件
    scene = moveit_commander.PlanningSceneInterface() # pyright: ignore[reportCallIssue]
    move_group = moveit_commander.MoveGroupCommander("arm") # pyright: ignore[reportCallIssue]

    move_group.set_max_velocity_scaling_factor(1.0)     # 调整为最大速度的 100%
    move_group.set_max_acceleration_scaling_factor(1.0) # 调整为最大加速度的 100%
    
    # 设置允许的误差
    move_group.set_goal_position_tolerance(0.005)
    move_group.set_goal_orientation_tolerance(0.01)

    # --- 1. 设置安全区域 ---
    # 定义边界
    bbox = {
        'y': [-0.5, 0.5],
        'z': [0.05, 0.6]
    }
    # 实例化并添加墙壁
    safety_manager = SafetyZoneManager(scene, frame_id=move_group.get_planning_frame())
    safety_manager.add_safety_walls(bbox)
    
    # --- 2. 回到初始位置 ---
    joint_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    move_group.set_joint_value_target(joint_goal)
    success = move_group.go(wait=True)
    rospy.loginfo(f"Reset arm success: {success}")
    print_current_pose(move_group)

    rospy.sleep(1)

    # --- 3. 测试: 移动到目标点，法兰盘正对 EYE_POSITION ---
    rospy.loginfo("\n--- TEST: Moving with flange facing EYE_POSITION ---")
    x, y, z = 0.4, 0.3, 0.4
    success = move_to_xyz_face_eye(
        move_group, x, y, z, 
        yaw_step=10.0, 
        yaw_levels=3, 
        pitch_step=5.0, 
        pitch_range=20.0
    )
    rospy.loginfo(f"移动结果: {'SUCCESS' if success else 'FAIL'}")
    print_current_pose(move_group)

    # --- 5. 清理场景 ---
    safety_manager.remove_safety_walls()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        move_robot_cartesian_demo()
    except rospy.ROSInterruptException:
        pass
