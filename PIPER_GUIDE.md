# 一、驱动机械臂
## 1.激活 USB-CAN 模块
终端输入以下命令进入实时监控模式：

```shell
sudo dmesg -w
```

插入,取出 USB 设备之后会看见usb端口号，找到对应的端口号，使用以下命令激活该端口模块

```shell
bash can_activate.sh can0 1000000 3-7:1.0
# 其中，波特率要设置为1000000,3-7:1.0代表USB总线3的第7号端口的第0个接口
```

## 2.启动 ROS 驱动节点

```shell
source devel/setup.bash
roscore
```

## 3.运行驱动节点，建立 PC 与机械臂硬件之间的双向通信

```shell
source devel/setup.bash
# 启动基础控制节点
roslaunch piper start_single_piper.launch can_port:=can0
```

## 4.验证机械臂状态

```shell
source ~/piper_ros/devel/setup.bash
# 查看实时状态反馈
rostopic echo /arm_status -n 1
```

关键参数说明：
- ctrl_mode: 必须为 1 才能接收控制指令。如果是 0（待机）或 2（示教），它不会动。
- arm_status: 必须为 0。如果是其他数字，说明有急停或报错。

## 5.使能机械臂（若机械臂已经使能则可以跳过）

```shell
source ~/piper_ros/devel/setup.bash
# 调用使能服务
rosservice call /enable_srv "enable_request: true"
```

## 6.测试归零运动
让它执行一个最安全的动作——回到原点。

```shell
rosservice call /go_zero_srv "is_mit_mode: false"
```

# 二、路径规划
## 1.启动硬件驱动与使能
路径规划的前提是 PC 必须能实时控制机械臂。
- 确保 roscore 正在运行之后启动 Piper 驱动。

```shell
source ~/piper_ros/devel/setup.bash
roslaunch piper start_single_piper.launch
```

- 使能机械臂（如果之前没开）：
```shell
rosservice call /enable_srv "enable_request: true"
```

## 2.启动 MoveIt

```shell
roslaunch piper_no_gripper_moveit demo.launch
```
如果启动成功，你会看到控制台输出 `You can start planning now!`。核心节点已成功与机械臂驱动建立连接。

## 3.手动拖动、命令行或脚本控制
### (1) 手动拖动
启动的是带 RViz 的 `demo.launch`，你会看到机械臂的 3D 模型。
- 选择规划组：在 RViz 左下角的 MotionPlanning 面板中，找到 Planning Group。
- 机械臂末端有一个彩色圆球和坐标轴。用鼠标左键拖动这些坐标轴，将虚拟模型移动到你想要的位置。
- 在 Planning 选项卡中，点击 Plan（查看规划出的轨迹虚影）。
- 确认轨迹安全后，点击 Execute。此时真实的机械臂会同步开始运动。

### (2)命令行

```shell
cd piper_ros
source devel/setup.bash
```

- 机械臂关节弧度控制

```shell
rosservice call /joint_moveit_ctrl_arm "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
max_velocity: 0.5
max_acceleration: 0.5" 
```

- 机械臂末端控制

```shell
rosservice call /joint_moveit_ctrl_endpose "joint_endpose: [0.099091, 0.008422, 0.246447, -0.09079689034052749, 0.7663049838381912, -0.02157924359457128, 0.6356625934370577]
max_velocity: 0.5
max_acceleration: 0.5" 
shell
```

### (3)脚本控制

```shell
cd piper_ros
source devel/setup.bash
rosrun moveit_ctrl joint_moveit_ctrl.py
```