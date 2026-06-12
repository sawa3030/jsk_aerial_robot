#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
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
    ref_alpha = math.radians(45.0)

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

    def get_wire_lengths(a1, a2):
        plus_long_wire = get_plus_pos_wire_length(a1, r_joint_2) + d + get_plus_pos_wire_length(a2, r_joint_2)
        minus_long_wire = get_minus_pos_wire_length(a1, r_joint_2) + d + get_minus_pos_wire_length(a2, r_joint_2)
        plus_short_wire = get_plus_pos_wire_length(a1, r_joint_2) + d
        minus_short_wire = get_minus_pos_wire_length(a1, r_joint_2) + d
        return plus_long_wire, minus_long_wire, plus_short_wire, minus_short_wire

    x_plus_long_wire, x_minus_long_wire, x_plus_short_wire, x_minus_short_wire = get_wire_lengths(alpha_1, alpha_2)
    ref_plus_long, ref_minus_long, ref_plus_short, ref_minus_short = get_wire_lengths(ref_alpha, ref_alpha)

    return (
        x_plus_long_wire - ref_plus_long,
        x_minus_long_wire - ref_minus_long,
        x_plus_short_wire - ref_plus_short,
        x_minus_short_wire - ref_minus_short,
    )


def get_angle_diff(wire_diff):
    # 4096 encoder ticks / 1 rev
    return int(wire_diff / r_wheel / math.pi * 4096)


class SoftJointToServoNode:
    # soft_airframe_202605 logical joints:
    # module1: soft_joint2,  soft_joint3
    # module2: soft_joint5,  soft_joint6
    # module3: soft_joint8,  soft_joint9
    # module4: soft_joint11, soft_joint12
    MODULE_JOINT_PAIRS = (
        ("soft_joint2", "soft_joint3"),
        ("soft_joint5", "soft_joint6"),
        ("soft_joint8", "soft_joint9"),
        ("soft_joint11", "soft_joint12"),
    )

    def __init__(self):
        rospy.init_node("soft_joint_to_servo")

        self.servo_center = 2047
        # 4 servos / module x 4 modules = 16 servos.info
        # module1: 0-3, module2: 4-7, module3: 8-11, module4: 12-15
        self.servo_indices = list(range(16))

        self.pub = rospy.Publisher("servo/target_states", ServoControlCmd, queue_size=1)
        self.sub = rospy.Subscriber("target_soft_joints_ctrl", JointState, self.cb, queue_size=1)

        rospy.loginfo("soft_joint_to_servo started.")

    def cb(self, msg: JointState):
        # joint_states から必要な soft joint を取り出す
        required_joints = [j for pair in self.MODULE_JOINT_PAIRS for j in pair]
        soft = {joint_name: None for joint_name in required_joints}

        for name, pos in zip(msg.name, msg.position):
            if name in soft:
                soft[name] = pos

        missing = [k for k, v in soft.items() if v is None]
        if missing:
            rospy.logwarn_throttle(1.0, f"Missing joints in joint_states: {missing}")
            return

        # 各moduleで wire/joint 対応を以下にそろえる:
        # y_plus_long  : joint{3*i-1}, joint{3*i} が正方向
        # y_minus_long : joint{3*i-1}, joint{3*i} が負方向
        # y_plus_short : joint{3*i} が正方向
        # y_minus_short: joint{3*i} が負方向
        # get_wire_diff(alpha_1, alpha_2) の alpha_1 が short 側に効くため、
        # alpha_1 <- pair後半(joint{3*i}), alpha_2 <- pair前半(joint{3*i-1}) とする。
        servo_vals = []
        for joint_3i_minus_1, joint_3i in self.MODULE_JOINT_PAIRS:
            alpha_1 = soft[joint_3i]  # short wire に効く側: joint{3*i}
            alpha_2 = soft[joint_3i_minus_1]
            y_plus_long, y_minus_long, y_plus_short, y_minus_short = get_wire_diff(alpha_1, alpha_2)
            servo_vals.extend(
                [
                    self.servo_center - get_angle_diff(y_plus_long),
                    self.servo_center - get_angle_diff(y_minus_long),
                    self.servo_center - get_angle_diff(y_minus_short),
                    self.servo_center - get_angle_diff(y_plus_short),
                ]
            )

        out = ServoControlCmd()
        out.index = list(self.servo_indices)
        out.angles = servo_vals
        self.pub.publish(out)

        rospy.loginfo_throttle(0.5, f"Published servo angles: {servo_vals}")
        # print(f"Published servo angles: {servo_vals}")


def main():
    try:
        SoftJointToServoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
