#!/usr/bin/env python3
"""将 reachable_range/points_*.txt 发布为 RViz Marker，按 g 刷新，ESC 退出。

用法:
    rosrun moveit_ctrl visualize_points.py                         # 交互式选择 (所有臂)
    rosrun moveit_ctrl visualize_points.py lower                   # 筛选 piper_lower 并选择
    rosrun moveit_ctrl visualize_points.py lower 0                 # 直接选 piper_lower 第 1 个文件
    rosrun moveit_ctrl visualize_points.py lower obs_x-0.7         # 筛选后按子串匹配
    rosrun moveit_ctrl visualize_points.py points_xxx.txt          # 直接指定文件 (全局 topic)
"""

import sys
import termios
import tty
import select
import threading
from pathlib import Path

import yaml
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


_SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DIR = _SCRIPT_DIR / "reachable_range"


def _resolve_can_port(arg):
    """短名称 → piper_<name>.yaml → can_port，否则直接返回 arg。"""
    path = Path(arg)
    if path.suffix not in ('.yaml', '.yml'):
        path = _SCRIPT_DIR / 'cfg' / f'piper_{arg}.yaml'
    if path.exists():
        cfg = yaml.safe_load(path.read_text())
        return cfg.get('arm', {}).get('can_port', arg)
    return arg


def _resolve_label(arg):
    """短名称 → YAML 文件 stem (如 lower → piper_lower)。"""
    path = Path(arg)
    if path.suffix not in ('.yaml', '.yml'):
        path = _SCRIPT_DIR / 'cfg' / f'piper_{arg}.yaml'
    if path.exists():
        return path.stem
    return arg


def _list_files(directory, arm_filter=None):
    """列出 points_*.txt，可筛选 arm_filter (如 piper_lower)。"""
    if arm_filter:
        return sorted(directory.glob(f"points_{arm_filter}_*.txt"))
    return sorted(directory.glob("points_*.txt"))


def _select_file(files):
    """编号 CLI 菜单。返回选中的 Path 或 None。"""
    print(f"\nFound {len(files)} file(s):")
    for i, f in enumerate(files):
        print(f"  [{i}] {f.name}")
    print()
    while True:
        try:
            choice = input(f"Select [0-{len(files) - 1}] or 'q' to quit: ").strip()
            if choice.lower() == 'q':
                return None
            idx = int(choice)
            if 0 <= idx < len(files):
                return files[idx]
        except (ValueError, EOFError):
            pass
        print("Invalid, try again.")


class KeyboardListener:
    """非阻塞键盘监听，按 g 刷新，按 ESC 退出。"""

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


def visualize_points(filename, can_port="", frame_id="dummy_link"):
    """发布点云到 RViz，按 g 刷新，按 ESC 退出。

    :param filename: points_*.txt 路径
    :param can_port: can_port 命名空间 (如 piper_lower)，用于 topic 前缀
    :param frame_id: RViz fixed frame
    """
    rospy.init_node('point_visualizer', anonymous=True)

    topic = f"{can_port}/workspace_visualization" if can_port else "workspace_visualization"
    pub = rospy.Publisher(topic, Marker, queue_size=10)

    rospy.loginfo(f"Reading points from '{filename}'...")

    points_list = []
    try:
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 3:
                    p = Point()
                    p.x = float(parts[0])
                    p.y = float(parts[1])
                    p.z = float(parts[2])
                    points_list.append(p)
    except FileNotFoundError:
        rospy.logerr(f"Error: File '{filename}' not found.")
        return

    rospy.loginfo(f"Loaded {len(points_list)} points. Publishing to /{topic} (frame={frame_id})...")

    clear = Marker()
    clear.header.frame_id = frame_id
    clear.ns = "reachable_space"
    clear.action = Marker.DELETEALL
    pub.publish(clear)
    rospy.sleep(0.1)

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "reachable_space"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.color.g = 1.0
    marker.color.a = 1.0
    marker.points = points_list

    for _ in range(3):
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rospy.sleep(0.2)
    rospy.loginfo("Done. Press 'g' to re-publish, ESC to exit.")

    kb = KeyboardListener()
    try:
        while not rospy.is_shutdown():
            ch = kb.poll()
            if ch:
                if ch == 'g':
                    for _ in range(3):
                        marker.header.stamp = rospy.Time.now()
                        pub.publish(marker)
                        rospy.sleep(0.1)
                    rospy.loginfo(f"Re-published {len(points_list)} points to /{topic}")
                elif ord(ch) == 27:
                    break
            rospy.sleep(0.05)
    finally:
        kb.restore()


def main():
    arg1 = sys.argv[1] if len(sys.argv) > 1 else ""
    arg2 = sys.argv[2] if len(sys.argv) > 2 else ""

    can_port = ""
    label = ""
    resolved_file = None

    if not arg1:
        # 无参数: 显示所有文件供选择
        files = _list_files(DEFAULT_DIR)
        if not files:
            print(f"Error: No points_*.txt files found in {DEFAULT_DIR}")
            sys.exit(1)
        if len(files) == 1:
            resolved_file = files[0]
        else:
            resolved_file = _select_file(files)
            if resolved_file is None:
                sys.exit(0)
                
        file_path = Path(resolved_file)
        file_name = file_path.name
        can_port = f"{file_name.split('_')[1]}_{file_name.split('_')[2]}"

    else:
        # 先检查是否为直接文件路径
        path = Path(arg1)
        if path.is_file():
            resolved_file = path
        elif (DEFAULT_DIR / arg1).is_file():
            resolved_file = DEFAULT_DIR / arg1
        else:
            # 作为 arm 名称解析
            can_port = _resolve_can_port(arg1)
            label = _resolve_label(arg1)

            files = _list_files(DEFAULT_DIR, arm_filter=label)
            if not files:
                print(f"Error: No points_{label}_*.txt files found in {DEFAULT_DIR}")
                sys.exit(1)

            if len(files) == 1:
                resolved_file = files[0]
            elif arg2:
                # 第二参数: 尝试作为数字索引
                try:
                    idx = int(arg2)
                    if 0 <= idx < len(files):
                        resolved_file = files[idx]
                    else:
                        print(f"Error: Index {idx} out of range [0, {len(files) - 1}]")
                        sys.exit(1)
                except ValueError:
                    # 尝试子串匹配
                    matches = [f for f in files if arg2 in f.name]
                    if not matches:
                        print(f"Error: No file matching '{arg2}' in filtered set")
                        sys.exit(1)
                    if len(matches) == 1:
                        resolved_file = matches[0]
                    else:
                        print(f"Multiple matches for '{arg2}':")
                        resolved_file = _select_file(matches)
                        if resolved_file is None:
                            sys.exit(0)
            else:
                # 无第二参数: 编号菜单
                resolved_file = _select_file(files)
                if resolved_file is None:
                    sys.exit(0)

    tag = f"can_port={can_port}" if can_port else "global"
    print(f"Visualizing: {resolved_file}  ({tag})")
    try:
        visualize_points(str(resolved_file), can_port=can_port)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
