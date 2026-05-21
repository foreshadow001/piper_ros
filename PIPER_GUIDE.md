# 终端一：驱动机械臂
## 1. 查看 USB-CAN 模块对应的 USB 端口

```shell
sudo ethtool -i can0 | grep bus
# can0: piper_lower, can1: piper_upper
bash can_activate.sh piper_upper 1000000 "3-3.2:1.0"
```

此处的意思是，1-2:1.0硬件编码的usb端口插入的can设备，名字被**重命名**为can_piper,波特率为1000000，并激活

## 2.运行驱动节点，建立 PC 与机械臂硬件之间的双向通信

```shell
source devel/setup.bash
# 启动基础控制节点
roslaunch piper start_single_piper.launch can_port:=piper_upper auto_enable:=true
```

# 终端二：路径规划服务器

```shell
source devel/setup.bash
roslaunch piper start_single_piper.launch can_port:=piper_upper gripper_val_mutiple:=2
```

# 终端三：路径规划节点

```shell
roslaunch piper_no_gripper_moveit demo.launch can_port:=piper_upper
```
如果启动成功，你会看到控制台输出 `You can start planning now!`。核心节点已成功与机械臂驱动建立连接。

## 终端四：脚本控制

```shell
source devel/setup.bash
rosrun moveit_ctrl piper_arm_controller.py upper
```