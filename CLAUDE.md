# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS Noetic (Ubuntu 20.04) catkin workspace for the AgileX Piper 6-DOF robotic arm. Communicates with physical hardware via CAN bus (`piper_sdk`). Upstream: `agilexrobotics/piper_ros` on branch `noetic`. This repo's `dev` branch adds workspace analysis, obstacle-aware MoveIt planning, and a safety-wall system.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

System deps: `python-can`, `piper_sdk` (pip), `ros-noetic-ruckig`, `ros-noetic-moveit` (apt). If MoveIt compilation fails, install via apt and delete `src/piper_moveit/moveit-1.1.11/` (it's already excluded via `CATKIN_IGNORE`).

## Package Architecture

| Package | Role |
|---------|------|
| `piper` | Core driver node. CAN bus ↔ ROS bridge. Publishes joint states, arm status, end-effector pose. Exposes enable/stop/reset/go-zero services. |
| `piper_msgs` | Custom messages (`PiperStatusMsg`, `PosCmd`, `PiperEulerPose`) and services (`Enable`, `Gripper`, `GoZero`). |
| `piper_description` | URDF/XACRO models (multiple firmware versions), meshes, RViz configs. With/without gripper, left/right hand variants. |
| `piper_moveit/moveit_ctrl` | **User-developed.** ROS service server + client for controlling the arm via MoveIt. Supports joint-space, Cartesian end-pose, and gripper commands. |
| `piper_moveit/piper_no_gripper_moveit` | MoveIt config package (arm only, 6 joints, group `arm`). |
| `piper_moveit/piper_with_gripper_moveit` | MoveIt config package (arm + gripper, groups `arm`, `gripper`, `piper`). |
| `piper_sim/piper_gazebo` | Gazebo simulation with ROS controllers. |
| `piper_sim/piper_mujoco` | MuJoCo simulation with PID/ROS control scripts. |
| `piper_noetic` | Empty placeholder package. |

## Key ROS Interfaces (Runtime)

**Topics published by driver (`piper_ctrl_single_node.py`):**
- `/joint_states_single` — actual joint positions (7 values: joint1-6 + gripper)
- `/arm_status` — `PiperStatusMsg` with ctrl_mode, arm_status, error codes
- `/end_pose` — `PoseStamped` end-effector pose (quaternion)
- `/end_pose_euler` — `PiperEulerPose` end-effector pose (Euler angles)

**Topics subscribed by driver:**
- `/joint_states` — command target: 7 values (joint1-6 rad, gripper 0-0.07m). `joint7` drives both physical gripper joints.

**Services provided by driver:**
- `/enable_srv`, `/stop_srv`, `/reset_srv`, `/go_zero_srv`, `/block_arm`, `/gripper_srv`

**Services provided by `joint_moveit_ctrl_server.py` (MoveIt bridge):**
- `/joint_moveit_ctrl_arm` — joint-space arm control (6 joints)
- `/joint_moveit_ctrl_gripper` — gripper control
- `/joint_moveit_ctrl_piper` — combined arm + gripper (7 values)
- `/joint_moveit_ctrl_endpose` — Cartesian end-pose control (7 values: xyz + quaternion)

## URDF Version Caveat

Firmware **S-V1.6-3 and later** shifts J2/J3 coordinate systems by 2 degrees. The default `piper_description.urdf` matches the newer firmware. Use `piper_description_old.urdf` for pre-S-V1.6-3 firmware.

## Runtime Workflow

1. Activate CAN: `bash can_activate.sh can0 1000000` (add USB bus-info as 3rd arg for multi-CAN setups)
2. Start driver: `roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true`
3. Enable arm: `rosservice call /enable_srv "enable_request: true"`
4. For MoveIt: `roslaunch piper_no_gripper_moveit demo.launch` (starts `joint_moveit_ctrl_server` automatically)

## User's Custom Features (dev branch)

All in commit `7404b8c` "Add MoveIt! Planning with obstacle":

- **[workspace_analyzer.py](src/piper_moveit/moveit_ctrl/scripts/workspace_analyzer.py):** Samples a 3D volume at configurable resolution, checks each point for IK reachability via MoveIt with optional safety-wall constraints. Uses geometric pre-filtering and aggressive planning timeouts for speed. Outputs reachable point clouds to text files.
- **[visualize_points.py](src/piper_moveit/moveit_ctrl/scripts/visualize_points.py):** Publishes workspace analysis results as RViz `Marker.POINTS` for visual inspection.
- **[joint_moveit_ctrl.py:178-465](src/piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl.py#L178-L465):** Extended with `SafetyZoneManager`, `move_to_xyz()`, `move_to_xyz_smart()` (auto-Yaw + iterative Pitch search), and a Cartesian demo with safety walls.
- **[PIPER_GUIDE.md](PIPER_GUIDE.md):** User's condensed quick-start guide in Chinese.
- Removed `moveit-1.1.11/` source tree (~240K lines deleted) — relies on apt-installed MoveIt instead.

Note: `SafetyZoneManager` is duplicated in both `workspace_analyzer.py` and `joint_moveit_ctrl.py`. This is a known DRY violation that should be resolved by extracting the class to a shared module if either file is modified.
