# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS Noetic (Ubuntu 20.04) catkin workspace for the AgileX Piper 6-DOF robotic arm. Communicates with physical hardware via CAN bus (`piper_sdk`). Upstream: `agilexrobotics/piper_ros` on branch `noetic`. This repo's `dev` branch adds multi-arm support (upper/lower namespace), a unified Cartesian arm controller, workspace analysis, safety-wall MoveIt planning, and pose monitoring tools.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

Unit tests for the moveit_ctrl search/fast_solve logic (stub backends, no hardware; needs `source devel/setup.bash` so `moveit_ctrl.srv` imports):

```bash
python3 -m pytest src/piper_moveit/moveit_ctrl/scripts/tests/ -v
```

System deps: `python-can`, `piper_sdk` (pip), `ros-noetic-ruckig`, `ros-noetic-moveit` (apt). MoveIt source `src/piper_moveit/moveit-1.1.11/` is excluded via `CATKIN_IGNORE` — install via apt instead.

## Package Architecture

| Package | Role |
|---------|------|
| `piper` | Core driver node. CAN bus ↔ ROS bridge. Publishes joint states, arm status, end-effector pose. Exposes enable/stop/reset/go-zero services. |
| `piper_msgs` | Custom messages (`PiperStatusMsg`, `PosCmd`, `PiperEulerPose`) and services (`Enable`, `Gripper`, `GoZero`). |
| `piper_description` | URDF/XACRO models (multiple firmware versions), meshes, RViz configs. With/without gripper, left/right hand variants. |
| `piper_moveit/moveit_ctrl` | **User-developed.** Contains the unified arm controller, MoveIt bridge server/client, workspace analyzer, pose monitors, Windows TCP bridge, and safety-wall logic. This is where most custom code lives. |
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
- Arm-specific config lives in YAML files: [piper_upper.yaml](src/piper_moveit/moveit_ctrl/scripts/cfg/piper_upper.yaml) and [piper_lower.yaml](src/piper_moveit/moveit_ctrl/scripts/cfg/piper_lower.yaml) — eye position, planning timeouts/search steps, safety bbox, obstacles (incl. camera FOV keep-out volumes), workspace sampling params, tool offset, arm-in-camera pose

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
- All use `JointMoveitCtrl.srv`; a `planning_time` field (>0) overrides the server's `~planning_time` param per request

**Intermediary topic:**
- `/joint_states_actual` — published by `joint_state_filter.py`; strips `gripper` from `/joint_states_single`

## Unified Arm Controller (`piper_arm_controller.py`)

The central Cartesian control class. See [piper_arm_controller.py](src/piper_moveit/moveit_ctrl/scripts/piper_arm_controller.py).

**Dual mode:**
- **Service mode** (`use_service=True`): sends Cartesian goals to the MoveIt bridge server. For controlling the real arm.
- **Direct mode** (`use_service=False`): in-process MoveIt (namespaced via `can_port`). For workspace analysis and batch planning.

**Key APIs:**
- `from_yaml("upper", use_service=...)` — loads `arm`, `safety_bbox`, `obstacles`, `workspace_analysis` from `cfg/piper_upper.yaml` (short name or full path)
- All public move/query/cleanup methods are serialized via an `RLock` (`@_synchronized` decorator) — sharing one instance across threads is safe
- `search_orientation(x, y, z, try_pose, abort_check)` — the **single** orientation search loop; both runtime modes and `workspace_analyzer` are thin `try_pose` backends (service+verify / plan+execute / plan-only), so sim and real share identical candidate order and abort semantics
- `fast_solve` — mode-derived (service mode: on; direct/analysis mode: off, no YAML key). Neighbors within `fast_solve.NEIGHBOR_RADIUS` (`resolution·√3`) of the target are inverse-distance-averaged into a single (Δα, Δβ) that **re-anchors** the spiral (whole spiral translated); the ideal-anchor spiral always follows as fallback, deduped. Reads `reachable_range/offsets_<arm>_<config_label>.txt`; falls back to pure spiral when the file is missing/stale (filename embeds `get_config_label()`) or no neighbor passes the radius gate
- `move_to(x, y, z)` — (α, β) spiral search so the flange Z-axis faces `eye_position` (γ=0); ideal angles computed geometrically, then widened by `yaw_step`/`pitch_step` until planning succeeds
- `move_to_with_orientation(x, y, z, α, β, γ)` — explicit Z-X-Z' target, no search
- `move_to_joints([6 rad values])` — joint-space move
- `get_current_pose()` / `get_current_zxz()` / `print_current_pose()` — pose queries

**Service-mode behaviors (non-obvious):**
- All service calls have hard timeouts (`wait_for_service` 3 s, call 10 s via `_call_srv_with_timeout`); a timeout raises `ServiceCallTimeout` (subclass of `rospy.ServiceException`), and inside `move_to`'s search loop it **aborts the whole search** instead of retrying the remaining candidates
- Drives the bridge server's planning timeout via `rospy.set_param("<ns>/joint_moveit_ctrl_server/planning_time", ...)` so unreachable poses fail in ~0.05 s instead of blocking
- Verifies success by reading `/<ns>/end_pose`: target reached only if distance < 0.02 m
- Safety walls (6 faces of `safety_bbox`) and obstacles are added via `PlanningSceneInterface` (lazy-init, 2 s timeout). If the scene can't be initialized, the constructor **raises RuntimeError** — the arm never moves without collision constraints (fail-fast by design)

Note: the `tool` and `arm_in_ccs` YAML entries are consumed by [end_pose_monitor.py](src/piper_moveit/moveit_ctrl/scripts/end_pose_monitor.py), **not** by the controller.

## Workspace Analysis

**[workspace_analyzer.py](src/piper_moveit/moveit_ctrl/scripts/workspace_analyzer.py):** Refactored to use `PiperArmController` direct mode. The reachability check calls the **same** `search_orientation` loop as runtime `move_to` (plan-only backend), so "reachable in analysis" ≡ "move_to succeeds on the real arm" (up to execution-layer differences). Samples a 3D volume with optional safety-wall constraints; uses aggressive planning timeouts. Outputs reachable point clouds **and per-point (Δα, Δβ) offset sidecars** (`offsets_*.txt`, format defined by [fast_solve.py](src/piper_moveit/moveit_ctrl/scripts/fast_solve.py)) consumed by `fast_solve`.

**[run_workspace_analyzer.launch](src/piper_moveit/moveit_ctrl/launch/run_workspace_analyzer.launch):** Starts its own **non-namespaced** `move_group` (execution disabled) plus the analyzer; pass `arm:=upper|lower` to select the config.

**[visualize_points.py](src/piper_moveit/moveit_ctrl/scripts/visualize_points.py):** Publishes workspace analysis results as RViz `Marker.POINTS` for visual inspection.

## Pose Monitoring Tools

**[end_pose_monitor.py](src/piper_moveit/moveit_ctrl/scripts/end_pose_monitor.py):** Real-time end-effector flange/tool pose monitor with RViz visualization (coordinate axis markers, X=red Y=green Z=blue). Supports namespaced operation via YAML short name. Keyboard: `g` print current pose, `t` toggle visualization (flange/tool/both), `ESC` quit.

**[dual_pose_monitor.py](src/piper_moveit/moveit_ctrl/scripts/dual_pose_monitor.py):** Monitors both `piper_upper` and `piper_lower` simultaneously. `u` prints upper pose, `l` prints lower pose, `ESC` quit. Uses Z-X-Z' Euler representation for pose display.

## Windows TCP Bridge

For teleoperation from a Windows eyetracker host. Network config in [net.yaml](src/piper_moveit/moveit_ctrl/scripts/cfg/net.yaml): Ubuntu `192.168.10.4`, Windows `192.168.10.3`.

- **[piper_windows_ctrl_server.py](src/piper_moveit/moveit_ctrl/scripts/piper_windows_ctrl_server.py)** (port 49301): control server. Creates one service-mode `PiperArmController` per arm, then serves line-based commands over TCP:
  - `READY` → `ACK` (handshake)
  - `MOVE_TO:<arm>:<x>,<y>,<z>` → `MOVED:<arm>:<x>,<y>,<z>,<qx>,<qy>,<qz>,<qw>,<α>,<β>,<γ>` or `ERROR:<arm>:no_solution`
  - `MOVE_JOINTS:<arm>:<j1>,...,<j6>` → `MOVED:...`; if MoveIt finds no path because the arm is already at the target, reports current pose instead of failing
  - `GET_POSE:<arm>` → `POSE:<arm>:...` (no movement)
  - `SHUTDOWN` → `SHUTDOWN_ACK`, exits the node
- **[windows_end_monitor.py](src/piper_moveit/moveit_ctrl/scripts/windows_end_monitor.py)** (port 49300): read-only pose server. Subscribes to both `/<can_port>/end_pose` topics and answers `GET_POSE:<upper|lower>` from cached poses.

Both responses use Z-X-Z' Euler angles in degrees.

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

**Windows bridge (needs both arms' `demo.launch` running):**
```bash
rosrun moveit_ctrl piper_windows_ctrl_server.py   # control server, port 49301
rosrun moveit_ctrl windows_end_monitor.py         # pose server, port 49300
```

**Workspace analysis (no namespace, own move_group):**
```bash
roslaunch moveit_ctrl run_workspace_analyzer.launch arm:=upper
rosrun moveit_ctrl visualize_points.py upper      # interactive file picker; 'g' re-publish, ESC quit
```

## Known Issues

- [joint_moveit_ctrl.py](src/piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl.py) is a legacy demo/prototype script (old yaw-pitch search, 4-wall `SafetyZoneManager`). Production wall logic is `PiperArmController._add_safety_walls()` (6 walls, from `safety_bbox`). Don't copy from it.
- [joint_moveit_ctrl_server.py](src/piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl_server.py) calls the driver's `/block_arm` — blocked by default at startup, unblocked only around each move — so the fake controller's residual target pose can't drive the arm on its own (see commit `29ae46c` "Fix No Move ERROR").
- `piper_arm_controller.py` and `workspace_analyzer.py` suppress MoveIt planning TIMED_OUT warnings by setting `ROSCONSOLE_CONFIG_FILE` (→ `scripts/rosconsole_no_warn.config`) at import time. This must happen before `roscpp` initializes (i.e., before any MoveIt import via rospy).
- The `gripper` joint name must match exactly in `joint_state_filter.py` (default: `"gripper"`). Configurable via `~strip_joints` param.
