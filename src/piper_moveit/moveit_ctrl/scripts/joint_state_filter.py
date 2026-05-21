#!/usr/bin/env python3
"""Relay /joint_states_single → /joint_states_actual, stripping 'gripper' joint."""

import rospy
from sensor_msgs.msg import JointState


class JointStateFilter:
    def __init__(self):
        self._pub = rospy.Publisher("joint_states_actual", JointState, queue_size=10)
        # Allow target topic and strip list to be configured via params
        self._strip = rospy.get_param("~strip_joints", ["gripper"])
        self._sub = rospy.Subscriber(
            "joint_states_single", JointState, self._callback, queue_size=10, tcp_nodelay=True
        )
        rospy.loginfo(f"JointStateFilter: stripping {self._strip}")

    def _callback(self, msg):
        out = JointState()
        out.header = msg.header
        for name, pos in zip(msg.name, msg.position):
            if name not in self._strip:
                out.name.append(name)
                out.position.append(pos)
        if out.name:
            self._pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("joint_state_filter")
    JointStateFilter()
    rospy.spin()
