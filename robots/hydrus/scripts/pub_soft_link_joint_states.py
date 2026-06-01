#!/usr/bin/env python

import math

import rospy
from sensor_msgs.msg import JointState


class PubSoftLinkJointStates(object):
    # From hydrus/robots/quad/soft_airframe_202605/robot.urdf.xacro
    # (soft_link_module i=1..4 -> actuated soft joints: s1,s2 for each module)
    SOFT_JOINT_NAMES = [
        "soft_joint2",
        "soft_joint3",
        "soft_joint5",
        "soft_joint6",
        "soft_joint8",
        "soft_joint9",
        "soft_joint11",
        "soft_joint12",
    ]

    def __init__(self):
        rospy.init_node("pub_soft_link_joint_states")

        self.target_deg = rospy.get_param("~target_deg", 45.0)
        self.publish_hz = rospy.get_param("~publish_hz", 500.0)
        self.target_rad = math.radians(self.target_deg)
        self.soft_joint_names = list(self.SOFT_JOINT_NAMES)

        print("is_simulation: ", self.is_simulation())

        if self.is_simulation():
            self.joint_states_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
            self.joint_control_topic_name = "joints_ctrl"
        else:
            self.joint_states_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
            self.joint_control_topic_name = "joint_states"

        rospy.loginfo(
            "publish %s soft joints to %s at %.3f deg (%.6f rad), %.2f Hz",
            len(self.soft_joint_names),
            self.joint_control_topic_name,
            self.target_deg,
            self.target_rad,
            self.publish_hz,
        )

    def run(self):
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = list(self.soft_joint_names)
            msg.position = [self.target_rad] * len(self.soft_joint_names)
            self.joint_states_pub.publish(msg)
            rate.sleep()

    def is_simulation(self) -> bool:
        return rospy.get_param('/use_sim_time', False)


if __name__ == "__main__":
    node = PubSoftLinkJointStates()
    node.run()
