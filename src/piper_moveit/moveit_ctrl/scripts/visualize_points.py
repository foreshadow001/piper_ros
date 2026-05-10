#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys

def visualize_points(filename, frame_id="dummy_link"):
    """
    读取 TXT 文件并在 RViz 中发布 Marker 点云。
    :param filename: 包含 XYZ 坐标的 TXT 文件路径
    :param frame_id: Marker 发布的坐标系 (应为机械臂的基座)
    """
    rospy.init_node('point_visualizer', anonymous=True)
    rospy.on_shutdown(lambda: rospy.loginfo("Visualizer stopped by Ctrl+C"))
    
    # 创建一个 Marker Publisher
    pub = rospy.Publisher('/workspace_visualization', Marker, queue_size=10)
    
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

    rospy.loginfo(f"Loaded {len(points_list)} points. Publishing to /workspace_visualization...")

    # 创建 Marker 消息
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "reachable_space"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    # 设置点的大小
    marker.scale.x = 0.02  # 点的宽度
    marker.scale.y = 0.02  # 点的高度

    # 设置点的颜色 (绿色，不透明)
    marker.color.g = 1.0
    marker.color.a = 1.0

    marker.points = points_list
    
    # 持续发布，确保 RViz 能够接收到
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    # 默认读取 'reachable_points.txt'，也可以从命令行参数传入文件名
    if len(sys.argv) > 1:
        file_to_visualize = sys.argv[1]
    else:
        file_to_visualize = "points_no_obs.txt"
        
    try:
        visualize_points(file_to_visualize)
    except rospy.ROSInterruptException:
        pass