#!/usr/bin/env python

import math

import rospy
from sensor_msgs.msg import JointState

JOINT_NAMES = ["soft_joint2", "soft_joint3", "soft_joint4", "soft_joint5", "soft_joint12", "soft_joint13", "soft_joint14", "soft_joint15",]
DEFAULT_START_DEG = 22.5
STEP_DEG = 0.5/2
CONFIRM_EVERY_DEG = 2.5
PUBLISH_INTERVAL_SEC = 0.2


def publish_joint_states(pub, angle_deg):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = JOINT_NAMES
    msg.position = [math.radians(angle_deg)] * len(JOINT_NAMES)
    pub.publish(msg)
    rospy.loginfo("publish angle: %.1f deg (%s)", angle_deg, ", ".join(JOINT_NAMES))


def read_input(prompt):
    try:
        return input(prompt)  # Python 2
    except NameError:
        return input(prompt)      # Python 3


def read_start_angle_deg():
    while True:
        raw = read_input("start angle [deg] (default: %.1f): " % DEFAULT_START_DEG).strip()
        if raw == "":
            return DEFAULT_START_DEG
        try:
            return float(raw)
        except ValueError:
            rospy.logwarn("invalid input: '%s' (please input a number)", raw)


def main():
    rospy.init_node("pub_soft_link_joint_states_simple")
    pub = rospy.Publisher("target_soft_joints_ctrl", JointState, queue_size=1)
    rospy.sleep(0.2)

    start_deg = read_start_angle_deg()
    angle_deg = start_deg
    publish_joint_states(pub, angle_deg)

    while not rospy.is_shutdown():
        angle_deg += STEP_DEG
        publish_joint_states(pub, angle_deg)
        rospy.sleep(PUBLISH_INTERVAL_SEC)

        if (angle_deg - start_deg) % CONFIRM_EVERY_DEG == 0:
            cmd = read_input("continue? [y to continue, q to quit]: ").strip().lower()
            if cmd == "q" or cmd == "":
                break


if __name__ == "__main__":
    main()
