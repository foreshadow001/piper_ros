#!/usr/bin/env python3

import rospy
import moveit_commander
import os
import sys
import math
import numpy as np
from tqdm import tqdm
import logging
import signal

from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_about_axis, quaternion_multiply

# 1. 设置日志级别：屏蔽 MoveIt 的繁杂输出，只显示严重错误
logging.getLogger("moveit_ros.robot_state").setLevel(logging.ERROR)
logging.getLogger("moveit_ros.planning_scene_monitor").setLevel(logging.ERROR)
logging.getLogger("ros.moveit_ros_planning_interface").setLevel(logging.ERROR)
logging.getLogger("moveit_ros.planning_interface").setLevel(logging.ERROR)
logging.getLogger("moveit_ros.planning_pipeline").setLevel(logging.ERROR)
logging.getLogger("moveit_ros.move_group").setLevel(logging.ERROR)

class SafetyZoneManager:
    """管理 MoveIt 规划场景中的安全边界墙"""
    def __init__(self, scene, frame_id="dummy_link"):
        self.scene = scene
        self.frame_id = frame_id
        self.wall_names = [
            "safety_wall_floor", "safety_wall_ceiling",
            "safety_wall_y_pos", "safety_wall_y_neg"
        ]

    def add_safety_walls(self, bbox):
        # 移除旧墙
        self.remove_safety_walls()
        rospy.sleep(0.2) # 稍微减少等待时间

        WALL_THICKNESS = 0.01
        WALL_LENGTH_X = 5.0 # 无限长
        
        # 解析障碍物空间
        y_min, y_max = bbox['y']
        z_min, z_max = bbox['z']
        y_range = y_max - y_min
        z_range = z_max - z_min

        # 1. 地板 (z = z_min)
        self._add_box(self.wall_names[0], 0, y_min + y_range/2, z_min - WALL_THICKNESS/2, 
                      (WALL_LENGTH_X, y_range, WALL_THICKNESS))
        # 2. 天花板 (z = z_max)
        self._add_box(self.wall_names[1], 0, y_min + y_range/2, z_max + WALL_THICKNESS/2, 
                      (WALL_LENGTH_X, y_range, WALL_THICKNESS))
        # 3. Y+ 墙
        self._add_box(self.wall_names[2], 0, y_max + WALL_THICKNESS/2, z_min + z_range/2, 
                      (WALL_LENGTH_X, WALL_THICKNESS, z_range))
        # 4. Y- 墙
        self._add_box(self.wall_names[3], 0, y_min - WALL_THICKNESS/2, z_min + z_range/2, 
                      (WALL_LENGTH_X, WALL_THICKNESS, z_range))

        rospy.sleep(0.5) 

    def _add_box(self, name, x, y, z, size):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.scene.add_box(name, pose, size=size)

    def remove_safety_walls(self):
        for wall_name in self.wall_names:
            self.scene.remove_world_object(wall_name)

class WorkspaceAnalyzer:
    def __init__(self, sampling_box, obstacle_box, resolution, with_obstacles=True):
        # 初始化 ROS
        if not rospy.core.is_initialized():
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node('workspace_analyzer_fast', anonymous=True)

        self.robot = moveit_commander.RobotCommander() # pyright: ignore[reportCallIssue]
        self.scene = moveit_commander.PlanningSceneInterface() # pyright: ignore[reportCallIssue]
        self.move_group = moveit_commander.MoveGroupCommander("arm") # pyright: ignore[reportCallIssue]

        # --- 关键优化 2: 极速 MoveIt 配置 ---
        # 对于存在性检测，如果0.05秒算不出来，那大概率就是算不出来了
        self.move_group.set_planning_time(0.05) 
        self.move_group.set_num_planning_attempts(1)
        self.move_group.set_planner_id("RRTConnect") # 通常 RRTConnect 速度最快

        self.sampling_box = sampling_box
        self.obstacle_box = obstacle_box
        self.resolution = resolution
        self.with_obstacles = with_obstacles
        self.reachable_points = []
        
        # 注册 Ctrl+C 信号处理 (可选，ROS本身也有处理机制)
        self.keep_running = True
        signal.signal(signal.SIGINT, self.signal_handler)

        self.safety_manager = SafetyZoneManager(self.scene, frame_id=self.move_group.get_planning_frame())
        
        suffix = "with_obs" if with_obstacles else "no_obs"
        self.output_file = f"points_{suffix}.txt"

    def signal_handler(self, sig, frame):
        rospy.logwarn("Ctrl+C detected! Stopping analysis...")
        self.keep_running = False

    def get_hybrid_quaternion(self, yaw_deg, pitch_deg, roll_deg):
        # 简单的姿态生成
        q_yaw = quaternion_about_axis(math.radians(yaw_deg), (0, 0, 1))
        q_pitch = quaternion_about_axis(math.radians(pitch_deg), (0, 1, 0))
        q_roll = quaternion_about_axis(math.radians(roll_deg), (1, 0, 0))
        return list(quaternion_multiply(quaternion_multiply(q_yaw, q_pitch), q_roll))

    def is_geometrically_safe(self, x, y, z):
        """
        关键优化 1: 几何预检查
        在调用 MoveIt 之前，用简单的数学判断点是否在障碍物框外。
        如果在墙外，直接 Fail，不需要规划。
        """
        if not self.with_obstacles:
            return True
        
        # 检查 Y 轴
        y_min, y_max = self.obstacle_box['y']
        if y < y_min or y > y_max:
            return False
            
        # 检查 Z 轴
        z_min, z_max = self.obstacle_box['z']
        if z < z_min or z > z_max:
            return False
            
        return True

    def check_reachability(self, x, y, z):
        # 1. 几何快速筛选 (O(1) 复杂度)
        if not self.is_geometrically_safe(x, y, z):
            return False

        # 2. Yaw 角度筛选
        auto_yaw_deg = math.degrees(math.atan2(y, x))
        if not (-90.0 <= auto_yaw_deg <= 90.0):
            return False

        # 3. MoveIt 规划
        # 减少候选姿态数量以提高速度，通常 90度(垂直) 和 45度 能覆盖大部分情况
        pitch_candidates = [90.0, 45.0, 0.0]
        
        for pitch in pitch_candidates:
            # 关键优化 3: 检查中断信号
            if rospy.is_shutdown() or not self.keep_running:
                return False

            q = self.get_hybrid_quaternion(auto_yaw_deg, pitch, 0.0)
            
            target_pose = Pose()
            target_pose.position.x = x; target_pose.position.y = y; target_pose.position.z = z
            target_pose.orientation.x = q[0]; target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]; target_pose.orientation.w = q[3]

            self.move_group.set_pose_target(target_pose)
            
            # plan() 返回 (success, ...)
            result = self.move_group.plan()
            is_success = result[0] if isinstance(result, tuple) else result
            
            if is_success:
                self.move_group.clear_pose_targets()
                return True
        
        self.move_group.clear_pose_targets()
        return False

    def analyze(self):
        try:
            if self.with_obstacles:
                rospy.loginfo(f"Building Obstacles: {self.obstacle_box}")
                self.safety_manager.add_safety_walls(self.obstacle_box)
            else:
                self.safety_manager.remove_safety_walls()

            # 生成采样点
            x_range = np.arange(self.sampling_box['x'][0], self.sampling_box['x'][1] + 0.001, self.resolution)
            y_range = np.arange(self.sampling_box['y'][0], self.sampling_box['y'][1] + 0.001, self.resolution)
            z_range = np.arange(self.sampling_box['z'][0], self.sampling_box['z'][1] + 0.001, self.resolution)
            
            total_points = len(x_range) * len(y_range) * len(z_range)
            rospy.loginfo(f"Sampling Box: {self.sampling_box}")
            rospy.loginfo(f"Start Analyzing {total_points} points...")

            with tqdm(total=total_points, unit="pt") as pbar:
                for x in x_range:
                    for y in y_range:
                        for z in z_range:
                            # 关键优化 3: 循环内检查退出信号
                            if rospy.is_shutdown() or not self.keep_running:
                                rospy.logwarn("Analysis interrupted by user.")
                                return # 直接退出到 finally 块保存数据

                            if self.check_reachability(x, y, z):
                                self.reachable_points.append((x, y, z))
                            pbar.update(1)

        except KeyboardInterrupt:
            rospy.logwarn("Keyboard Interrupt captured.")
        
        finally:
            # 无论如何都保存已分析的数据
            if self.with_obstacles:
                self.safety_manager.remove_safety_walls()
            self.save_results()
            moveit_commander.roscpp_shutdown()

    def save_results(self):
        if os.path.exists(self.output_file):
            os.remove(self.output_file)
        with open(self.output_file, 'w') as f:
            for p in self.reachable_points:
                f.write(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
        rospy.loginfo(f"Saved {len(self.reachable_points)} points to {self.output_file}")

if __name__ == '__main__':
    # --- 配置区域 ---
    # 1. 采样空间 (机器人尝试去的地方)
    SAMPLING_BOX = {
        'x': [0.3, 0.6],
        'y': [-0.5, 0.5],
        'z': [0.1, 0.6]
    }
    
    # 2. 障碍空间 (安全墙的位置, X无限)
    OBSTACLE_BOX = {
        'y': [-0.6, 0.6],
        'z': [0.05, 0.65]
    }

    RESOLUTION = 0.05 # 5cm 分辨率

    WITH_OBSTACLES = True # 是否包含障碍物

    try:
        analyzer = WorkspaceAnalyzer(SAMPLING_BOX, OBSTACLE_BOX, RESOLUTION, with_obstacles=WITH_OBSTACLES)
        analyzer.analyze()
    except Exception as e:
        print(f"Error: {e}")