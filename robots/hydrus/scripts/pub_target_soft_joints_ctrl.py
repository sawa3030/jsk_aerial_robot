#!/usr/bin/env python3

import math

import rospy
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "soft_joint2",
    "soft_joint3",
    "soft_joint4",
    "soft_joint5",
    "soft_joint7",
    "soft_joint8",
    "soft_joint9",
    "soft_joint10",
    "soft_joint12",
    "soft_joint13",
    "soft_joint14",
    "soft_joint15",
    "soft_joint17",
    "soft_joint18",
    "soft_joint19",
    "soft_joint20",
]


def main():
    rospy.init_node("pub_target_soft_joints_ctrl")

    topic = rospy.get_param("~topic", "target_soft_joints_ctrl")
    publish_hz = float(rospy.get_param("~publish_hz", 10.0))
    angle_deg = float(rospy.get_param("~angle_deg", 22.5))

    if publish_hz <= 0.0:
        rospy.logfatal("~publish_hz must be greater than zero")
        return

    publisher = rospy.Publisher(topic, JointState, queue_size=1)
    rate = rospy.Rate(publish_hz)
    positions = [math.radians(angle_deg)] * len(JOINT_NAMES)

    rospy.loginfo(
        "Publishing %d soft-joint targets (%.1f deg) to %s at %.1f Hz",
        len(JOINT_NAMES),
        angle_deg,
        topic,
        publish_hz,
    )

    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(JOINT_NAMES)
        msg.position = list(positions)
        publisher.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
