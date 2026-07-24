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


def get_plus_pos_wire_length(alpha, r_joint, divide_num):
    if alpha == 0:
        return s + d

    half_divide_num = divide_num / 2.0
    r = (s - d * (half_divide_num - 1.0)) / abs(alpha)
    if alpha > 0:
        return (
            divide_num * (r - r_joint - 1.5) * math.sin(abs(alpha) / divide_num)
            + half_divide_num * d
        )
    return (
        divide_num * (r + r_joint + 1.5) * math.sin(abs(alpha) / divide_num)
        + half_divide_num * d
    )


def get_minus_pos_wire_length(alpha, r_joint, divide_num):
    if alpha == 0:
        return s + d

    half_divide_num = divide_num / 2.0
    r = (s - d * (half_divide_num - 1.0)) / abs(alpha)
    if alpha > 0:
        return (
            divide_num * (r + r_joint + 1.5) * math.sin(abs(alpha) / divide_num)
            + half_divide_num * d
        )
    return (
        divide_num * (r - r_joint - 1.5) * math.sin(abs(alpha) / divide_num)
        + half_divide_num * d
    )


def get_wire_diff(alpha_1, alpha_2, alpha_3, alpha_4, divide_num=4):
    ref_alpha = math.radians(22.5)

    def get_wire_lengths(a1, a2, a3, a4):
        # Servo contribution per module:
        # 4*i + 0: bend soft_joint(5*i+2..5) in plus direction
        # 4*i + 1: bend soft_joint(5*i+2..5) in minus direction
        # 4*i + 2: bend soft_joint(5*i+4..5) in minus direction
        # 4*i + 3: bend soft_joint(5*i+4..5) in plus direction
        plus_long_wire = (
            get_plus_pos_wire_length(a1, r_joint_2, divide_num)
            + d
            + get_plus_pos_wire_length(a2, r_joint_2, divide_num)
            + d
            + get_plus_pos_wire_length(a3, r_joint_2, divide_num)
            + d
            + get_plus_pos_wire_length(a4, r_joint_2, divide_num)
        )
        minus_long_wire = (
            get_minus_pos_wire_length(a1, r_joint_2, divide_num)
            + d
            + get_minus_pos_wire_length(a2, r_joint_2, divide_num)
            + d
            + get_minus_pos_wire_length(a3, r_joint_2, divide_num)
            + d
            + get_minus_pos_wire_length(a4, r_joint_2, divide_num)
        )
        plus_short_wire = (
            get_plus_pos_wire_length(a3, r_joint_2, divide_num)
            + d
            + get_plus_pos_wire_length(a4, r_joint_2, divide_num)
        )
        minus_short_wire = (
            get_minus_pos_wire_length(a3, r_joint_2, divide_num)
            + d
            + get_minus_pos_wire_length(a4, r_joint_2, divide_num)
        )
        return plus_long_wire, minus_long_wire, plus_short_wire, minus_short_wire

    x_plus_long_wire, x_minus_long_wire, x_plus_short_wire, x_minus_short_wire = get_wire_lengths(
        alpha_1, alpha_2, alpha_3, alpha_4
    )
    ref_plus_long, ref_minus_long, ref_plus_short, ref_minus_short = get_wire_lengths(
        ref_alpha, ref_alpha, ref_alpha, ref_alpha
    )

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
    # module1: soft_joint2, soft_joint3, soft_joint4,  soft_joint5
    # module2: soft_joint7, soft_joint8, soft_joint9,  soft_joint10
    # module3: soft_joint12,soft_joint13,soft_joint14, soft_joint15
    # module4: soft_joint17,soft_joint18,soft_joint19, soft_joint20
    MODULE_JOINT_GROUPS = (
        ("soft_joint2", "soft_joint3", "soft_joint4", "soft_joint5"),
        ("soft_joint7", "soft_joint8", "soft_joint9", "soft_joint10"),
        ("soft_joint12", "soft_joint13", "soft_joint14", "soft_joint15"),
        ("soft_joint17", "soft_joint18", "soft_joint19", "soft_joint20"),
    )
    MODULE_FREE_JOINT_GROUPS = (
        ("soft_joint2", "soft_joint3", "soft_joint4", "soft_joint5"),
        ("soft_joint12", "soft_joint13", "soft_joint14", "soft_joint15"),
    )
    FREE_SERVO_TARGET = 8000

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
        required_joints = [j for group in self.MODULE_JOINT_GROUPS for j in group]
        free_joints = {j for group in self.MODULE_FREE_JOINT_GROUPS for j in group}
        free_groups = set(self.MODULE_FREE_JOINT_GROUPS)
        soft = {joint_name: None for joint_name in required_joints}
        print(f"Received joint_states: {msg.name}, {msg.position}")

        # free module 側の joint が publish されてきた場合はこのメッセージを無視する
        # if any(name in free_joints for name in msg.name):
        #     rospy.logwarn_throttle(1.0, "Received free joints in target_soft_joints_ctrl. Skip publishing.")
        #     return

        for name, pos in zip(msg.name, msg.position):
            if name in soft:
                soft[name] = pos

        # free module 側は target_soft_joints_ctrl に値が来ない前提なので欠損を許容
        missing = [k for k, v in soft.items() if v is None and k not in free_joints]
        if missing:
            rospy.logwarn_throttle(1.0, f"Missing joints in joint_states: {missing}")
            return

        # 5分割(1/8,1/4,1/4,1/4,1/8)で1モジュール4関節のため、
        # a1..a4 をそのまま wire モデルに渡して長さ差分を計算する。
        servo_vals = []
        for joint_group in self.MODULE_JOINT_GROUPS:
            if joint_group in free_groups:
                servo_vals.extend([self.FREE_SERVO_TARGET] * 4)
                continue

            q0, q1, q2, q3 = (soft[jn] for jn in joint_group)
            y_plus_long, y_minus_long, y_plus_short, y_minus_short = get_wire_diff(q0, q1, q2, q3)
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
