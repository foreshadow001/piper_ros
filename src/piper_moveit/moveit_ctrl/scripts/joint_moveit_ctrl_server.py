#!/usr/bin/env python

import rospy
from moveit_commander import *
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlResponse
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler

class JointMoveitCtrlServer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('joint_moveit_ctrl_server')

        # 关节状态频率守卫: CAN 断开后驱动仍低频发布冻结数据
        # (话题不死、stamp 照填), 必须按窗口频率检测。
        self._js_count = 0
        self._js_window_count = 0
        self._js_window_t0 = rospy.Time.now().to_sec()
        self._js_topic = rospy.resolve_name('joint_states_actual')
        self._js_sub = rospy.Subscriber(
            self._js_topic, JointState, self._js_cb, queue_size=1)

        # 初始化 MoveIt
        roscpp_initialize([])
        self.robot = RobotCommander()

        # 获取 MoveIt 规划组列表
        available_groups = self.robot.get_group_names()
        rospy.loginfo(f"Available MoveIt groups: {available_groups}")

        # 仅实例化存在的规划组
        self.arm_move_group = None
        self.gripper_move_group = None
        self.piper_move_group = None

        # 规划超时: 缩短默认 5s 以加速姿态搜索失败时的回退
        planning_time = rospy.get_param("~planning_time", 0.5)
        planning_attempts = rospy.get_param("~planning_attempts", 1)

        if "arm" in available_groups:
            self.arm_move_group = MoveGroupCommander("arm")
            self.arm_move_group.set_planning_time(planning_time)
            self.arm_move_group.set_num_planning_attempts(planning_attempts)
            rospy.loginfo(f"Initialized arm move group (planning_time={planning_time}s, attempts={planning_attempts}).")

        if "gripper" in available_groups:
            self.gripper_move_group = MoveGroupCommander("gripper")
            self.gripper_move_group.set_planning_time(planning_time)
            self.gripper_move_group.set_num_planning_attempts(planning_attempts)
            rospy.loginfo("Initialized gripper move group.")

        if "piper" in available_groups:
            self.piper_move_group = MoveGroupCommander("piper")
            self.piper_move_group.set_planning_time(planning_time)
            self.piper_move_group.set_num_planning_attempts(planning_attempts)
            rospy.loginfo("Initialized piper move group.")

        # 创建关节运动控制服务
        self.arm_srv = rospy.Service('joint_moveit_ctrl_arm', JointMoveitCtrl, self.handle_joint_moveit_ctrl_arm)
        self.gripper_srv = rospy.Service('joint_moveit_ctrl_gripper', JointMoveitCtrl, self.handle_joint_moveit_ctrl_gripper)
        self.piper_srv = rospy.Service('joint_moveit_ctrl_piper', JointMoveitCtrl, self.handle_joint_moveit_ctrl_piper)
        self.endpose_srv = rospy.Service('joint_moveit_ctrl_endpose', JointMoveitCtrl, self.handle_joint_moveit_ctrl_endpose)

        # 驱动节点 block_arm 服务代理 (阻止 CAN 指令发送，避免假控制器持续占线)
        self._block_srv = None
        try:
            rospy.wait_for_service('block_arm', timeout=3.0)
            self._block_srv = rospy.ServiceProxy('block_arm', SetBool)
            # 启动时立即阻断: 防止假控制器的默认位姿驱动机械臂自行运动
            self._block_srv(SetBool._request_class(data=True))
            rospy.loginfo("Connected to /block_arm service — arm blocked by default.")
        except rospy.ROSException:
            rospy.logwarn("/block_arm service not available — arm will hold position after each move.")

        rospy.loginfo("Joint MoveIt Control Services Ready.")

    def _js_cb(self, msg):
        self._js_count += 1

    def _joint_states_fresh(self, window=1.0, min_rate=10.0):
        """关节状态发布频率 ≥ min_rate → True。

        注: 不能用 header.stamp 判新鲜 — CAN 断开后驱动仍低频发布,
        stamp 填的是发送时刻 (内容是冻结的旧关节值), 时间戳是"新"的。
        必须按窗口内计数测频率 (正常 ~100Hz, 断链后 <1Hz)。
        """
        now = rospy.Time.now().to_sec()
        # 窗口滚动: 每过 window 秒重置计数
        if now - self._js_window_t0 >= window:
            self._js_window_count = self._js_count
            self._js_window_t0 = now
            self._js_count = 0
        rate = self._js_count / max(1e-6, now - self._js_window_t0)
        return rate >= min_rate

    def _guard_joint_states(self, arm_label):
        """运动前守卫: 关节状态频率异常时拒绝执行并返回失败。

        避免假执行 — CAN 断开后 move_group 对冻结的 fake controller
        规划+回灌照常返回成功, 真臂却没动, 客户端完全无感知。
        """
        if not self._joint_states_fresh():
            rospy.logerr(
                f"{arm_label} 拒绝执行: 关节状态话题频率异常 "
                f"(/{self._js_topic}, 低于 10Hz) — CAN 断开或驱动停止?")
            return False
        return True

    def _apply_planning_params(self, move_group, request):
        """应用规划超时。优先使用请求中的 planning_time (需 catkin_make 后生效)。"""
        planning_time = (request.planning_time
                         if hasattr(request, 'planning_time') and request.planning_time > 0
                         else rospy.get_param("~planning_time", 0.5))
        planning_attempts = rospy.get_param("~planning_attempts", 1)
        move_group.set_planning_time(planning_time)
        move_group.set_num_planning_attempts(planning_attempts)

    def _begin_move(self, move_group):
        """解除 block → 同步假控制器当前位姿，消除上次目标的残留。"""
        if self._block_srv is not None:
            try:
                self._block_srv(SetBool._request_class(data=False))
            except rospy.ServiceException:
                pass
            current_joints = move_group.get_current_joint_values()
            move_group.set_joint_value_target(current_joints)
            move_group.go(wait=True)

    def _end_move(self):
        """恢复 block，阻止假控制器残留位姿下发到驱动。"""
        if self._block_srv is not None:
            try:
                self._block_srv(SetBool._request_class(data=True))
            except rospy.ServiceException:
                pass

    def handle_joint_moveit_ctrl_arm(self, request):
        rospy.loginfo("Received arm joint movement request.")

        if not self._guard_joint_states('arm'):
            return JointMoveitCtrlResponse(status=False, error_code=2)
        try:
            if self.arm_move_group:
                self._apply_planning_params(self.arm_move_group, request)
                self._begin_move(self.arm_move_group)
                arm_joint_goal = request.joint_states[:6]
                self.arm_move_group.set_joint_value_target(arm_joint_goal)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.arm_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.arm_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.arm_move_group.go(wait=True)
                self._end_move()
                rospy.loginfo("Arm movement executed successfully.")
            else:
                rospy.logerr("Arm move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during arm movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_gripper(self, request):
        rospy.loginfo("Received gripper joint movement request.")

        if not self._guard_joint_states('gripper'):
            return JointMoveitCtrlResponse(status=False, error_code=2)
        try:
            if self.gripper_move_group:
                self._begin_move(self.gripper_move_group)
                gripper_goal = [request.gripper]
                self.gripper_move_group.set_joint_value_target(gripper_goal)
                self.gripper_move_group.go(wait=True)
                self._end_move()
                rospy.loginfo("Gripper movement executed successfully.")
            else:
                rospy.logerr("Gripper move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during gripper movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_piper(self, request):
        rospy.loginfo("Received piper joint movement request.")

        if not self._guard_joint_states('piper'):
            return JointMoveitCtrlResponse(status=False, error_code=2)
        try:
            if self.piper_move_group:
                self._apply_planning_params(self.piper_move_group, request)
                self._begin_move(self.piper_move_group)
                piper_joint_goal = list(request.joint_states[:6]) + [request.gripper]
                self.piper_move_group.set_joint_value_target(piper_joint_goal)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.piper_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.piper_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.piper_move_group.go(wait=True)
                self._end_move()
                rospy.loginfo("Piper movement executed successfully.")
            else:
                rospy.logerr("Piper move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during piper movement: {str(e)}")
        
        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_endpose(self, request):
        rospy.loginfo("Received endpose movement request.")

        if not self._guard_joint_states('endpose'):
            return JointMoveitCtrlResponse(status=False, error_code=2)
        try:
            if self.arm_move_group:
                self._apply_planning_params(self.arm_move_group, request)
                self._begin_move(self.arm_move_group)
                position = request.joint_endpose[:3]
                if len(request.joint_endpose) == 7:
                    # 四元数 [qx, qy, qz, qw]
                    quaternion = request.joint_endpose[3:]
                    rospy.loginfo("Using Quaternion for orientation: (qx, qy, qz, qw) -> %f, %f, %f, %f", *quaternion)
                else:
                    rospy.logerr("Invalid joint_endpose size. It must be 7 (Quaternion).")
                    return JointMoveitCtrlResponse(status=False, error_code=1)

                target_pose = Pose()
                target_pose.position.x = position[0]
                target_pose.position.y = position[1]
                target_pose.position.z = position[2]
                target_pose.orientation.x = quaternion[0]
                target_pose.orientation.y = quaternion[1]
                target_pose.orientation.z = quaternion[2]
                target_pose.orientation.w = quaternion[3]

                self.arm_move_group.set_pose_target(target_pose)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.arm_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.arm_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.arm_move_group.go(wait=True)
                self._end_move()
                rospy.loginfo("Endpose movement executed successfully.")
            else:
                rospy.logerr("Arm move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during endpose movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

if __name__ == '__main__':
    JointMoveitCtrlServer()
    rospy.spin()
