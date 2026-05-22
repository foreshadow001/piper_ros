#!/usr/bin/env python3
"""同时监听两台机械臂的末端法兰位姿，按 u/l 打印，按 ESC 退出。

用法:
    rosrun moveit_ctrl dual_pose_monitor.py

按键:
    u — 打印 piper_upper 法兰盘位姿
    l — 打印 piper_lower 法兰盘位姿
    ESC — 退出
"""

import math
import sys
import termios
import tty
import select
import threading

import rospy
from geometry_msgs.msg import PoseStamped


def _zxz_from_quaternion(qx, qy, qz, qw):
    """四元数 → 内旋 Z-X-Z' 欧拉角 (度)."""
    r02 = 2.0 * (qx * qz + qw * qy)
    r12 = 2.0 * (qy * qz - qw * qx)
    r20 = 2.0 * (qx * qz - qw * qy)
    r21 = 2.0 * (qy * qz + qw * qx)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    beta = math.acos(max(-1.0, min(1.0, r22)))
    sin_beta = math.sin(beta)

    if abs(sin_beta) > 1e-6:
        alpha = math.atan2(r02, -r12)
        gamma = math.atan2(r20, r21)
    else:
        gamma = 0.0
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qw * qz)
        alpha = math.atan2(-r01, r00) if beta < math.pi / 2.0 else math.atan2(r01, r00)

    return math.degrees(alpha), math.degrees(beta), math.degrees(gamma)


def pose_to_str(pose, label):
    if pose is None:
        return f"[{label}] no data yet"

    pos = pose.position
    ori = pose.orientation
    alpha, beta, gamma = _zxz_from_quaternion(ori.x, ori.y, ori.z, ori.w)
    return (
        f"\n[{label}] Position (xyz):     [{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}]  "
        f"\n[{label}] Orientation (wxyz): [{ori.w:.4f}, {ori.x:.4f}, {ori.y:.4f}, {ori.z:.4f}]  "
        f"\n[{label}] Euler Z-X-Z' (α-β-γ): [{alpha:.1f}°, {beta:.1f}°, {gamma:.1f}°]\n"
    )


class KeyboardListener:
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

    def restore(self):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)


class DualPoseMonitor:
    def __init__(self):
        self._pose_upper = None
        self._pose_lower = None
        self._lock = threading.Lock()

        self._sub_upper = rospy.Subscriber(
            'piper_upper/end_pose', PoseStamped,
            lambda m: self._cb(m, 'upper'))
        self._sub_lower = rospy.Subscriber(
            'piper_lower/end_pose', PoseStamped,
            lambda m: self._cb(m, 'lower'))

        rospy.loginfo("DualPoseMonitor: listening on /piper_upper/end_pose "
                      "and /piper_lower/end_pose")
        rospy.loginfo("Keys: u=upper  l=lower  ESC=exit")

    def _cb(self, msg, which):
        with self._lock:
            if which == 'upper':
                self._pose_upper = msg.pose
            else:
                self._pose_lower = msg.pose

    @property
    def pose_upper(self):
        with self._lock:
            return self._pose_upper

    @property
    def pose_lower(self):
        with self._lock:
            return self._pose_lower

    def print(self, which):
        if which == 'upper':
            rospy.loginfo(pose_to_str(self.pose_upper, "piper_upper"))
        else:
            rospy.loginfo(pose_to_str(self.pose_lower, "piper_lower"))


def main():
    rospy.init_node("dual_pose_monitor", anonymous=True)
    monitor = DualPoseMonitor()
    kb = KeyboardListener()

    try:
        while not rospy.is_shutdown():
            ch = kb.poll()
            if ch:
                if ch == 'u':
                    monitor.print('upper')
                elif ch == 'l':
                    monitor.print('lower')
                elif ord(ch) == 27:
                    rospy.loginfo("DualPoseMonitor: exit")
                    break
            rospy.sleep(0.05)
    finally:
        kb.restore()


if __name__ == '__main__':
    main()
