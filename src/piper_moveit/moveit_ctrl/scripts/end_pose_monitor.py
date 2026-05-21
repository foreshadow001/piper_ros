#!/usr/bin/env python3
"""实时监听机械臂末端法兰盘位姿，按 g 打印当前位姿，按 ESC 退出。

同时发布法兰盘坐标系可视化 Marker (X=红, Y=绿, Z=蓝) 到 RViz。

用法:
    rosrun moveit_ctrl end_pose_monitor.py              # 全局命名空间
    rosrun moveit_ctrl end_pose_monitor.py piper_lower  # 指定 can_port
    rosrun moveit_ctrl end_pose_monitor.py lower        # 从 YAML 短名称读取 can_port
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
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


_SCRIPT_DIR = Path(__file__).resolve().parent

AXIS_LENGTH = 0.16   # 坐标轴长度 (米)
AXIS_RADIUS = 0.002  # 箭头杆半径


def _resolve_can_port(arg):
    """短名称 → piper_<name>.yaml → can_port，否则直接返回 arg。"""
    path = Path(arg)
    if path.suffix not in ('.yaml', '.yml'):
        path = _SCRIPT_DIR / 'cfg' / f'piper_{arg}.yaml'
    if path.exists():
        cfg = yaml.safe_load(path.read_text())
        return cfg.get('arm', {}).get('can_port', arg)
    return arg


def _zxz_from_quaternion(qx, qy, qz, qw):
    """从四元数提取 Intrinsic Z-X-Z' 欧拉角 (经典欧拉角)。

    旋转过程 (内旋 / 绕体轴):
        1. 绕原始 Z 轴 旋转 α  (alpha)
        2. 绕新的  X' 轴 旋转 β  (beta)
        3. 绕新的  Z''轴 旋转 γ  (gamma)

    对应旋转矩阵: R = Rz(α) * Rx(β) * Rz(γ)

          ┌                                                  ┐
          │ cα·cγ - sα·cβ·sγ    -cα·sγ - sα·cβ·cγ    sα·sβ │
      R = │ sα·cγ + cα·cβ·sγ    -sα·sγ + cα·cβ·cγ   -cα·sβ │
          │      sβ·sγ                sβ·cγ             cβ   │
          └                                                  ┘

    反解:
        β  = acos(R[2,2])
        α  = atan2(R[0,2], -R[1,2])    (sinβ ≠ 0)
        γ  = atan2(R[2,0],  R[2,1])    (sinβ ≠ 0)

    返回 (α, β, γ) 单位: 度。
    """

    r02 = 2.0 * (qx * qz + qw * qy)          # sin(α)*sin(β)
    r12 = 2.0 * (qy * qz - qw * qx)          # -cos(α)*sin(β)
    r20 = 2.0 * (qx * qz - qw * qy)          # sin(β)*sin(γ)
    r21 = 2.0 * (qy * qz + qw * qx)          # sin(β)*cos(γ)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)    # cos(β)

    beta = math.acos(max(-1.0, min(1.0, r22)))
    sin_beta = math.sin(beta)

    if abs(sin_beta) > 1e-6:
        alpha = math.atan2(r02, -r12)
        gamma = math.atan2(r20, r21)
    else:
        # 万向节死锁: β ≈ 0 或 β ≈ π, Z 轴重合, α 与 γ 耦合
        gamma = 0.0
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qw * qz)
        if beta < math.pi / 2.0:
            alpha = math.atan2(-r01, r00)     # β ≈ 0
        else:
            alpha = math.atan2(r01, r00)      # β ≈ π

    return math.degrees(alpha), math.degrees(beta), math.degrees(gamma)


def _make_axis_marker(frame_id, ns, marker_id, pose, axis, color):
    """构造单轴 ARROW marker。"""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = ns
    m.id = marker_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose = pose

    # 箭头几何
    m.scale.x = AXIS_LENGTH   # 总长
    m.scale.y = AXIS_RADIUS * 4  # 箭头宽度
    m.scale.z = AXIS_RADIUS * 4

    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = 0.8

    # 将 axis 方向编码到 pose 姿态中 (ARROW 默认沿 +X)
    # 需要把 (1,0,0) 旋转到 axis 方向，然后乘上当前姿态
    from tf.transformations import quaternion_multiply, quaternion_about_axis

    default_dir = (1.0, 0.0, 0.0)
    # 计算把 default_dir 转到 axis 的四元数
    if abs(axis[0] - default_dir[0]) < 1e-6 and \
       abs(axis[1] - default_dir[1]) < 1e-6 and \
       abs(axis[2] - default_dir[2]) < 1e-6:
        q_align = [0.0, 0.0, 0.0, 1.0]
    elif abs(axis[0] + default_dir[0]) < 1e-6 and \
         abs(axis[1] + default_dir[1]) < 1e-6 and \
         abs(axis[2] + default_dir[2]) < 1e-6:
        q_align = quaternion_about_axis(math.pi, (0, 0, 1))
    else:
        cross = (
            default_dir[1] * axis[2] - default_dir[2] * axis[1],
            default_dir[2] * axis[0] - default_dir[0] * axis[2],
            default_dir[0] * axis[1] - default_dir[1] * axis[0],
        )
        dot = default_dir[0] * axis[0] + default_dir[1] * axis[1] + default_dir[2] * axis[2]
        angle = math.acos(max(-1.0, min(1.0, dot)))
        norm = math.sqrt(cross[0]*cross[0] + cross[1]*cross[1] + cross[2]*cross[2])
        if norm < 1e-9:
            q_align = [0.0, 0.0, 0.0, 1.0]
        else:
            s = math.sin(angle / 2.0) / norm
            q_align = [cross[0] * s, cross[1] * s, cross[2] * s, math.cos(angle / 2.0)]

    # 当前姿态四元数
    q_pose = [pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w]

    # 合成: 先 q_pose (基座→法兰盘), 再 q_align (法兰盘内转轴)
    q_final = quaternion_multiply(q_pose, q_align)
    m.pose.orientation.x = q_final[0]
    m.pose.orientation.y = q_final[1]
    m.pose.orientation.z = q_final[2]
    m.pose.orientation.w = q_final[3]

    return m


class KeyboardListener:
    """非阻塞键盘监听，线程安全。"""

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

    def get(self):
        with self._lock:
            ch, self._char = self._char, None
        return ch

    def restore(self):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)


class EndPoseMonitor:
    """监听 end_pose 话题，按 g 打印位姿，按 ESC 退出。

    同时发布法兰盘坐标系可视化 Marker:
      /{can_port}/flange_frame_x  (红, +X)
      /{can_port}/flange_frame_y  (绿, +Y)
      /{can_port}/flange_frame_z  (蓝, +Z)
    """

    def __init__(self, can_port=""):
        self._can_port = can_port
        self._pose = None
        self._frame_id = None
        self._lock = threading.Lock()

        topic = f"{can_port}/end_pose" if can_port else "end_pose"
        self._sub = rospy.Subscriber(topic, PoseStamped, self._callback)

        # 每个轴一个 publisher
        frame_ns = f"{can_port}/" if can_port else ""
        self._pubs = {
            'x': rospy.Publisher(f'{frame_ns}flange_frame_x', Marker, queue_size=1),
            'y': rospy.Publisher(f'{frame_ns}flange_frame_y', Marker, queue_size=1),
            'z': rospy.Publisher(f'{frame_ns}flange_frame_z', Marker, queue_size=1),
        }

        rospy.loginfo(f"EndPoseMonitor: listening on /{topic}, "
                      "press 'g' to print pose, ESC to exit")
        rospy.loginfo(f"Publishing flange frame to /{frame_ns}flange_frame_[xyz]")

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

    def publish_frame(self):
        """发布法兰盘坐标系 (X/Y/Z 轴 ARROW marker)。"""
        pose = self.pose
        if pose is None:
            return

        frame_id = self.frame_id
        ns = "flange_frame"
        now = rospy.Time.now()

        axes = [
            ('x', 0, (1.0, 0.0, 0.0), (1.0, 0.2, 0.2)),
            ('y', 1, (0.0, 1.0, 0.0), (0.2, 1.0, 0.2)),
            ('z', 2, (0.0, 0.0, 1.0), (0.2, 0.2, 1.0)),
        ]

        for name, mid, axis, color in axes:
            m = _make_axis_marker(frame_id, ns, mid, pose, axis, color)
            m.header.stamp = now
            self._pubs[name].publish(m)

    @staticmethod
    def pose_to_str(pose):
        if pose is None:
            return "no data yet"

        pos = pose.position
        ori = pose.orientation
        alpha, beta, gamma = _zxz_from_quaternion(ori.x, ori.y, ori.z, ori.w)

        return (
            f"\nPosition (xyz):     [{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}]  "
            f"\nOrientation (wxyz): [{ori.w:.4f}, {ori.x:.4f}, {ori.y:.4f}, {ori.z:.4f}]  "
            f"\nEuler Z-X-Z' (α-β-γ): [{alpha:.1f}°, {beta:.1f}°, {gamma:.1f}°]\n"
        )

    def print(self):
        rospy.loginfo(self.pose_to_str(self.pose))


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    can_port = _resolve_can_port(arg) if arg else ""
    ns_prefix = f" ({can_port})" if can_port else ""

    rospy.init_node("end_pose_monitor", anonymous=True)

    monitor = EndPoseMonitor(can_port)
    kb = KeyboardListener()

    # 发布帧率 (~10 Hz)，避免占用过高 CPU
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            ch = kb.poll()
            if ch:
                if ch == 'g':
                    monitor.print()
                elif ord(ch) == 27:  # ESC
                    rospy.loginfo(f"EndPoseMonitor{ns_prefix}: exit")
                    break
            monitor.publish_frame()
            rate.sleep()
    finally:
        kb.restore()


if __name__ == '__main__':
    main()
