# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS Noetic (Ubuntu 20.04) catkin workspace for the AgileX Piper 6-DOF robotic arm. Communicates with physical hardware via CAN bus (`piper_sdk`). Upstream: `agilexrobotics/piper_ros` on branch `noetic`. This repo's `dev` branch adds multi-arm support (upper/lower namespace), a unified Cartesian arm controller, workspace analysis, safety-wall MoveIt planning, and pose monitoring tools.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

System deps: `python-can`, `piper_sdk` (pip), `ros-noetic-ruckig`, `ros-noetic-moveit` (apt). MoveIt source `src/piper_moveit/moveit-1.1.11/` is excluded via `CATKIN_IGNORE` — install via apt instead.

## Package Architecture

| Package | Role |
|---------|------|
| `piper` | Core driver node. CAN bus ↔ ROS bridge. Publishes joint states, arm status, end-effector pose. Exposes enable/stop/reset/go-zero services. |
| `piper_msgs` | Custom messages (`PiperStatusMsg`, `PosCmd`, `PiperEulerPose`) and services (`Enable`, `Gripper`, `GoZero`). |
| `piper_description` | URDF/XACRO models (multiple firmware versions), meshes, RViz configs. With/without gripper, left/right hand variants. |
| `piper_moveit/moveit_ctrl` | **User-developed.** Contains the unified arm controller, MoveIt bridge server/client, workspace analyzer, pose monitors, and safety-wall logic. This is where most custom code lives. |
| `piper_moveit/piper_no_gripper_moveit` | MoveIt config package (arm only, 6 joints, group `arm`). |
| `piper_moveit/piper_with_gripper_moveit` | MoveIt config package (arm + gripper, groups `arm`, `gripper`, `piper`). |
| `piper_sim/piper_gazebo` | Gazebo simulation with ROS controllers. |
| `piper_sim/piper_mujoco` | MuJoCo simulation with PID/ROS control scripts. |
| `piper_noetic` | Empty placeholder package. |

## Multi-Arm Architecture (recent: commits `3950647`–`3846a0e`)

Two independent arms (`piper_upper`, `piper_lower`) run simultaneously by using the `can_port` as a ROS namespace:

- Each arm gets its own CAN port (`can_piper_upper`, `can_piper_lower`), own driver node in its namespace, and own MoveIt instance
- **`demo.launch`** wraps everything — MoveIt group, `joint_state_filter`, `joint_moveit_ctrl_server`, and CAN driver — inside `<group ns="$(arg can_port)">`. Pass `can_port:=piper_upper` or `can_port:=piper_lower`
- **`joint_state_filter.py`** strips the `gripper` joint from `/joint_states_single` → `/joint_states_actual` so MoveIt's joint_state_publisher only sees the 6 arm joints
- Arm-specific config lives in YAML files: [piper_upper.yaml](src/piper_moveit/moveit_ctrl/scripts/cfg/piper_upper.yaml) and [piper_lower.yaml](src/piper_moveit/moveit_ctrl/scripts/cfg/piper_lower.yaml) — eye position, safety bbox, obstacles, workspace params, tool offset, arm-in-camera pose

## Key ROS Interfaces (Runtime)

**Topics published by driver (`piper_ctrl_single_node.py`):**
- `/joint_states_single` — actual joint positions (7 values: joint1-6 + gripper)
- `/arm_status` — `PiperStatusMsg` with ctrl_mode, arm_status, error codes
- `/end_pose` — `PoseStamped` end-effector pose (quaternion)
- `/end_pose_euler` — `PiperEulerPose` end-effector pose (Z-X-Z' Euler angles)

**Topics subscribed by driver:**
- `/joint_states` — command target: 7 values (joint1-6 rad, gripper 0-0.07m)

**Services provided by driver:**
- `/enable_srv`, `/stop_srv`, `/reset_srv`, `/go_zero_srv`, `/block_arm`, `/gripper_srv`

**Services provided by `joint_moveit_ctrl_server.py` (MoveIt bridge):**
- `/joint_moveit_ctrl_arm` — joint-space arm control (6 joints)
- `/joint_moveit_ctrl_gripper` — gripper control
- `/joint_moveit_ctrl_piper` — combined arm + gripper (7 values)
- `/joint_moveit_ctrl_endpose` — Cartesian end-pose control (7 values: xyz + quaternion)

**Intermediary topic:**
- `/joint_states_actual` — published by `joint_state_filter.py`; strips `gripper` from `/joint_states_single`

## Unified Arm Controller (`piper_arm_controller.py`)

The central Cartesian control class. See [piper_arm_controller.py](src/piper_moveit/moveit_ctrl/scripts/piper_arm_controller.py) (873 lines).

**Dual mode:**
- **Service mode** (`use_service=True`): sends Cartesian goals to the MoveIt bridge server. For controlling the real arm.
- **Direct mode** (`use_service=False`): in-process MoveIt. For workspace analysis and batch planning.

**Key features:**
- YAML-driven config: `PiperArmController.from_yaml("upper", use_service=True)` loads all parameters from `cfg/piper_upper.yaml`
- Eye-in-hand coordinate transforms via `arm_in_ccs` YAML entry (Z-X-Z' rotation + translation)
- Tool offset support: flange → tool transform (rotation then translation, defined in YAML)
- `move_to(x, y, z)` — basic Cartesian target with default orientation
- `move_to_xyz_smart(x, y, z)` — auto-optimizes Yaw and iteratively searches Pitch
- Thread-safe with `_lock` for concurrent access

## Workspace Analysis

**[workspace_analyzer.py](src/piper_moveit/moveit_ctrl/scripts/workspace_analyzer.py):** Refactored to use `PiperArmController` direct mode. Samples a 3D volume, checks each point for IK reachability with optional safety-wall constraints. Uses geometric pre-filtering and aggressive planning timeouts. Outputs reachable point clouds to text files.

**[run_workspace_analyzer.launch](src/piper_moveit/moveit_ctrl/launch/run_workspace_analyzer.launch):** Launches workspace analysis in the correct namespace with config file argument.

**[visualize_points.py](src/piper_moveit/moveit_ctrl/scripts/visualize_points.py):** Publishes workspace analysis results as RViz `Marker.POINTS` for visual inspection.

## Pose Monitoring Tools

**[end_pose_monitor.py](src/piper_moveit/moveit_ctrl/scripts/end_pose_monitor.py):** Real-time end-effector flange/tool pose monitor with RViz visualization (coordinate axis markers, X=red Y=green Z=blue). Supports namespaced operation via YAML short name. Keyboard: `g` print current pose, `t` toggle visualization (flange/tool/both), `ESC` quit.

**[dual_pose_monitor.py](src/piper_moveit/moveit_ctrl/scripts/dual_pose_monitor.py):** Monitors both `piper_upper` and `piper_lower` simultaneously. `u` prints upper pose, `l` prints lower pose, `ESC` quit. Uses Z-X-Z' Euler representation for pose display.

## Coordinate Conventions

- Pose representation: **intrinsic Z-X-Z' Euler angles** (α, β, γ in degrees)
- Rotation application order: `R_z(α) * R_x(β) * R_z(γ)`
- Tool offset in config: rotation (Z-X-Z') applied first, then translation — all in flange frame
- `arm_in_ccs`: arm base pose in camera coordinate system (for eye-in-hand setups)

## URDF Version Caveat

Firmware **S-V1.6-3 and later** shifts J2/J3 coordinate systems by 2 degrees. The default `piper_description.urdf` matches the newer firmware. Use `piper_description_old.urdf` for pre-S-V1.6-3 firmware.

## Runtime Workflow

**Single arm:**
1. Activate CAN: `bash can_activate.sh can0 1000000` (add USB bus-info as 3rd arg for multi-CAN)
2. Start driver: `roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true`
3. Enable arm: `rosservice call /enable_srv "enable_request: true"`
4. For MoveIt + controller: `roslaunch piper_no_gripper_moveit demo.launch can_port:=piper_upper` (starts driver, MoveIt, filter, and bridge server automatically)

**Dual arm:**
1. Activate both CAN buses: `bash can_activate.sh piper_upper 1000000 "3-3.2:1.0"` + same for lower
2. Launch in 2 terminals:
   - `roslaunch piper_no_gripper_moveit demo.launch can_port:=piper_upper`
   - `roslaunch piper_no_gripper_moveit demo.launch can_port:=piper_lower`

**Controller script:**
```bash
rosrun moveit_ctrl piper_arm_controller.py upper   # service mode, reads cfg/piper_upper.yaml
```

**Pose monitoring:**
```bash
rosrun moveit_ctrl end_pose_monitor.py piper_upper  # or: upper
rosrun moveit_ctrl dual_pose_monitor.py              # both arms
```

## Known Issues

- `SafetyZoneManager` is duplicated in both [workspace_analyzer.py](src/piper_moveit/moveit_ctrl/scripts/workspace_analyzer.py) and [joint_moveit_ctrl.py](src/piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl.py). If either file needs changes, extract the class to a shared module first.
- `piper_arm_controller.py` suppresses MoveIt planning TIMED_OUT warnings by setting `ROSCONSOLE_CONFIG_FILE` at import time. This must happen before `roscpp` initializes (i.e., before any MoveIt import via rospy).
- The `gripper` joint name must match exactly in `joint_state_filter.py` (default: `"gripper"`). Configurable via `~strip_joints` param.
