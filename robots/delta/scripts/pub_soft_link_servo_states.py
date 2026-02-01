#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoControlCmd

# ---- Tail wire model params (あなたのスクリプトと同じ) ----
s = 230
d = 5
r_joint_2 = 85 / 2
r_wheel = 20


def get_wire_diff(alpha_1, alpha_2):
    divide_num = 8

    def get_plus_pos_wire_length(alpha, r_joint):
        if alpha == 0:
            return s + d
        r = (s - d * (divide_num / 2 - 1)) / abs(alpha)
        if alpha > 0:
            return (
                divide_num * (r - r_joint - 1.5) * math.sin(abs(alpha) / divide_num)
                + divide_num / 2 * d
            )
        else:
            return (
                divide_num * (r + r_joint + 1.5) * math.sin(abs(alpha) / divide_num)
                + divide_num / 2 * d
            )

    def get_minus_pos_wire_length(alpha, r_joint):
        if alpha == 0:
            return s + d
        r = (s - d * (divide_num / 2 - 1)) / abs(alpha)
        if alpha > 0:
            return (
                divide_num * (r + r_joint + 1.5) * math.sin(abs(alpha) / divide_num)
                + divide_num / 2 * d
            )
        else:
            return (
                divide_num * (r - r_joint - 1.5) * math.sin(abs(alpha) / divide_num)
                + divide_num / 2 * d
            )

    x_plus_long_wire = (
        get_plus_pos_wire_length(alpha_1, r_joint_2) + d + get_plus_pos_wire_length(alpha_2, r_joint_2)
    )
    x_minus_long_wire = (
        get_minus_pos_wire_length(alpha_1, r_joint_2) + d + get_minus_pos_wire_length(alpha_2, r_joint_2)
    )
    x_plus_short_wire = get_plus_pos_wire_length(alpha_1, r_joint_2) + d
    x_minus_short_wire = get_minus_pos_wire_length(alpha_1, r_joint_2) + d

    return (
        x_plus_long_wire - (s + d) * 2 - d,
        x_minus_long_wire - (s + d) * 2 - d,
        x_plus_short_wire - (s + d) - d,
        x_minus_short_wire - (s + d) - d,
    )


def get_angle_diff(wire_diff):
    # 4096 encoder ticks / 1 rev
    return int(wire_diff / r_wheel / math.pi * 4096)


class SoftJointToServoNode:
    def __init__(self):
        rospy.init_node("soft_joint_to_servo")

        self.servo_center = 2047

        self.pub = rospy.Publisher("servo/target_states", ServoControlCmd, queue_size=1)
        self.sub = rospy.Subscriber("joint_states", JointState, self.cb, queue_size=1)

        rospy.loginfo("soft_joint_to_servo started.")

    def cb(self, msg: JointState):
        # joint_states から必要な soft joint を取り出す
        soft = {
            "soft_joint2": None,
            "soft_joint3": None,
            "soft_joint8": None,
            "soft_joint9": None,
        }

        for name, pos in zip(msg.name, msg.position):
            if name in soft:
                soft[name] = pos

        missing = [k for k, v in soft.items() if v is None]
        if missing:
            rospy.logwarn_throttle(1.0, f"Missing joints in joint_states: {missing}")
            return

        s2 = soft["soft_joint2"]
        s3 = soft["soft_joint3"]
        s8 = soft["soft_joint8"]
        s9 = soft["soft_joint9"]

        # ---- 指定の対応関係 ----
        # (soft_joint2, soft_joint3) -> servo 8,7,9,10
        alpha_1_a = s2
        alpha_2_a = s3

        # (-soft_joint9, -soft_joint8) -> servo 12,11,13,14  ※順番と符号
        alpha_1_b = -s9
        alpha_2_b = -s8

        print(alpha_1_a, alpha_2_a, alpha_1_b, alpha_2_b)

        # ---- wire diff ----
        x_plus_long_a, x_minus_long_a, x_plus_short_a, x_minus_short_a = get_wire_diff(alpha_1_a, alpha_2_a)
        x_plus_long_b, x_minus_long_b, x_plus_short_b, x_minus_short_b = get_wire_diff(alpha_1_b, alpha_2_b)

        # ---- servo ticks (あなたの式と同じ) ----
        servo_a = [
            self.servo_center + get_angle_diff(x_plus_long_a),
            self.servo_center - get_angle_diff(x_minus_long_a),
            self.servo_center + get_angle_diff(x_plus_short_a),
            self.servo_center - get_angle_diff(x_minus_short_a),
        ]
        servo_b = [
            self.servo_center + get_angle_diff(x_plus_long_b),
            self.servo_center - get_angle_diff(x_minus_long_b),
            self.servo_center + get_angle_diff(x_plus_short_b),
            self.servo_center - get_angle_diff(x_minus_short_b),
        ]

        servo_vals = servo_a + servo_b

        out = ServoControlCmd()
        out.index = [8, 7, 9, 10, 12, 11, 13, 14]
        out.angles = servo_vals
        self.pub.publish(out)

        rospy.loginfo_throttle(0.5, f"Published servo angles: {servo_vals}")


def main():
    try:
        SoftJointToServoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
