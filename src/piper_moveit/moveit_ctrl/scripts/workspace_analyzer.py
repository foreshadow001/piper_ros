#!/usr/bin/env python3

import os
from pathlib import Path

# 抑制 MoveIt 规划 TIMED_OUT 刷屏 WARN (必须在 roscpp 初始化前设置)
_SCRIPT_DIR = Path(__file__).resolve().parent
_CONFIG = str(_SCRIPT_DIR / 'rosconsole_no_warn.config')
if os.path.exists(_CONFIG) and 'ROSCONSOLE_CONFIG_FILE' not in os.environ:
    os.environ['ROSCONSOLE_CONFIG_FILE'] = _CONFIG

import sys
import yaml
import rospy
import math
import numpy as np
from tqdm import tqdm  # type: ignore
import logging
import signal

from geometry_msgs.msg import Pose  # type: ignore
from tf.transformations import quaternion_about_axis, quaternion_multiply  # type: ignore

logging.getLogger("moveit_ros.robot_state").setLevel(logging.ERROR)
logging.getLogger("moveit_ros.planning_scene_monitor").setLevel(logging.ERROR)
logging.getLogger("ros.moveit_ros_planning_interface").setLevel(logging.ERROR)
logging.getLogger("moveit_ros.planning_interface").setLevel(logging.ERROR)
logging.getLogger("moveit_ros.planning_pipeline").setLevel(logging.ERROR)
logging.getLogger("moveit_ros.move_group").setLevel(logging.ERROR)

OUTPUT_DIR = _SCRIPT_DIR / "reachable_range"


class WorkspaceAnalyzer:
    """对采样空间进行 IK 可达性分析，结果保存到 reachable_range/ 目录。"""

    def __init__(self, controller, sampling_box, resolution=0.05, label=None):
        """
        :param controller: 已配置好的 PiperArmController 实例 (含 safety_bbox / obstacles)
        :param sampling_box: {'x': [min,max], 'y': [min,max], 'z': [min,max]}
        :param resolution: 采样分辨率 (米)
        :param label: 机械臂名称标签 (如 piper_upper)，用于输出文件名前缀
        """
        if not rospy.core.is_initialized():
            rospy.init_node('workspace_analyzer', anonymous=True)

        self.controller = controller
        self.sampling_box = sampling_box
        self.resolution = resolution
        self.reachable_points = []

        self.keep_running = True
        signal.signal(signal.SIGINT, self.signal_handler)

        self.move_group = self.controller.move_group
        self.move_group.set_planner_id("RRTConnect")

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        config_label = self.controller.get_config_label()
        if label:
            self.output_file = os.path.join(OUTPUT_DIR, f"points_{label}_{config_label}.txt")
        else:
            self.output_file = os.path.join(OUTPUT_DIR, f"points_{config_label}.txt")

        rospy.loginfo(f"WorkspaceAnalyzer 已初始化: sampling={sampling_box}, "
                      f"safety_bbox={controller.safety_bbox}, obstacles={len(controller.obstacles)}, "
                      f"output={self.output_file}")

    def signal_handler(self, sig, frame):
        rospy.logwarn("Ctrl+C detected! Stopping analysis...")
        self.keep_running = False

    @staticmethod
    def _quaternion_from_zxz(alpha_deg, beta_deg, gamma_deg):
        """Intrinsic Z-X-Z' (α → β → γ): R = Rz(α) * Rx(β) * Rz(γ)."""
        q_z1 = quaternion_about_axis(math.radians(alpha_deg), (0, 0, 1))
        q_x  = quaternion_about_axis(math.radians(beta_deg),  (1, 0, 0))
        q_z2 = quaternion_about_axis(math.radians(gamma_deg), (0, 0, 1))
        return list(quaternion_multiply(quaternion_multiply(q_z1, q_x), q_z2))

    def is_geometrically_safe(self, x, y, z):
        bbox = self.controller.safety_bbox
        if bbox is None:
            return True
        for axis, val in (('x', x), ('y', y), ('z', z)):
            limits = bbox.get(axis)
            if limits is None:
                continue
            if val < limits[0] or val > limits[1]:
                return False
        return True

    def check_reachability(self, x, y, z):
        if not self.is_geometrically_safe(x, y, z):
            return False

        # α 使法兰盘 Z 轴大致指向目标在 XY 平面的投影方向
        auto_alpha_deg = math.degrees(math.atan2(x, -y))
        if not (0.0 <= auto_alpha_deg <= 180.0):
            return False

        beta_candidates = [90.0, 45.0, 0.0]

        for beta in beta_candidates:
            if rospy.is_shutdown() or not self.keep_running:
                return False

            q = self._quaternion_from_zxz(auto_alpha_deg, beta, 0.0)

            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]

            self.move_group.set_pose_target(target_pose)
            result = self.move_group.plan()
            is_success = result[0] if isinstance(result, tuple) else result

            if is_success:
                self.move_group.clear_pose_targets()
                return True

        self.move_group.clear_pose_targets()
        return False

    def analyze(self):
        try:
            x_range = np.arange(self.sampling_box['x'][0], self.sampling_box['x'][1] + 0.001, self.resolution)
            y_range = np.arange(self.sampling_box['y'][0], self.sampling_box['y'][1] + 0.001, self.resolution)
            z_range = np.arange(self.sampling_box['z'][0], self.sampling_box['z'][1] + 0.001, self.resolution)

            total_points = len(x_range) * len(y_range) * len(z_range)
            rospy.loginfo(f"Sampling Box: {self.sampling_box}")
            rospy.loginfo(f"Start Analyzing {total_points} points...")

            with tqdm(total=total_points, unit="pt", dynamic_ncols=True) as pbar:
                for x in x_range:
                    for y in y_range:
                        for z in z_range:
                            if rospy.is_shutdown() or not self.keep_running:
                                rospy.logwarn("Analysis interrupted by user.")
                                return

                            if self.check_reachability(x, y, z):
                                self.reachable_points.append((x, y, z))
                            pbar.update(1)

        except KeyboardInterrupt:
            rospy.logwarn("Keyboard Interrupt captured.")

        finally:
            self.save_results()
            self.controller.clear_obstacles()

    def save_results(self):
        with open(self.output_file, 'w') as f:
            for p in self.reachable_points:
                f.write(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
        rospy.loginfo(f"Saved {len(self.reachable_points)} points to {self.output_file}")


if __name__ == '__main__':
    from piper_arm_controller import PiperArmController

    # 独立运行时自动加载 robot_description (与 demo.launch 中 planning_context.launch 一致)
    if not rospy.core.is_initialized():
        rospy.init_node('workspace_analyzer', anonymous=True)

    if not rospy.has_param('robot_description'):
        import rospkg
        _rp = rospkg.RosPack()
        _urdf = os.path.join(_rp.get_path('piper_description'),
                             'urdf', 'piper_no_gripper_description.urdf')
        _srdf = os.path.join(_rp.get_path('piper_no_gripper_moveit'),
                             'config', 'piper_description.srdf')
        _jl   = os.path.join(_rp.get_path('piper_no_gripper_moveit'),
                             'config', 'joint_limits.yaml')
        _cl   = os.path.join(_rp.get_path('piper_no_gripper_moveit'),
                             'config', 'cartesian_limits.yaml')
        _kin  = os.path.join(_rp.get_path('piper_no_gripper_moveit'),
                             'config', 'kinematics.yaml')

        rospy.set_param('robot_description', open(_urdf).read())
        rospy.set_param('robot_description_semantic', open(_srdf).read())
        rospy.set_param('robot_description_planning/joint_limits', yaml.safe_load(open(_jl)))
        rospy.set_param('robot_description_planning/cartesian_limits', yaml.safe_load(open(_cl)))
        rospy.set_param('robot_description_kinematics', yaml.safe_load(open(_kin)))
        rospy.loginfo(f"Loaded robot_description from {_urdf}")

    _name = sys.argv[1] if len(sys.argv) > 1 else rospy.get_param('~arm', 'upper')
    try:
        ctrl = PiperArmController.from_yaml(_name)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)

    wa = ctrl.workspace_config
    label = Path(ctrl.yaml_path).stem
    analyzer = WorkspaceAnalyzer(
        ctrl, wa['sampling_box'], wa.get('resolution', 0.05), label=label,
    )
    analyzer.analyze()
