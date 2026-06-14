#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates
from scipy.optimize import minimize

# ---- Tail wire model params (pub_soft_link_servo_states.py と同じ) ----
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


def angle_tick_to_wire_diff(tick_diff):
    # get_angle_diff() の逆変換
    return float(tick_diff) * math.pi * r_wheel / 4096.0


class EstimateJointStatesNode:
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

    # module1: 0-3, module2: 4-7, module3: 8-11, module4: 12-15
    MODULE_SERVO_INDICES = (
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (8, 9, 10, 11),
        (12, 13, 14, 15),
    )

    def __init__(self):
        rospy.init_node("estimate_joint_states")

        self.servo_center = int(rospy.get_param("~servo_center", 2047))
        self.max_joint_abs_rad = float(rospy.get_param("~max_joint_abs_rad", 1.5))
        self.w_prev = float(rospy.get_param("~weight_previous", 0.02))
        self.minimize_maxiter = int(rospy.get_param("~ik_max_iters", 80))

        servo_topic = rospy.get_param("~servo_states_topic", "servo/states")
        joint_topic = rospy.get_param("~joint_states_topic", "joint_states")

        self.pub = rospy.Publisher(joint_topic, JointState, queue_size=1)
        self.sub = rospy.Subscriber(servo_topic, ServoStates, self.cb, queue_size=1)

        # 初期値は 45deg (forward script の ref_alpha と整合)
        q0 = math.radians(45.0)
        self.prev_est = [[q0, q0] for _ in range(len(self.MODULE_JOINT_PAIRS))]

        rospy.loginfo(
            "estimate_joint_states started. sub=%s, pub=%s",
            servo_topic,
            joint_topic,
        )

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _residual_sq(a1, a2, measured):
        y_plus_long, y_minus_long, y_plus_short, y_minus_short = get_wire_diff(a1, a2)
        return (
            (y_plus_long - measured[0]) ** 2
            + (y_minus_long - measured[1]) ** 2
            + (y_minus_short - measured[2]) ** 2
            + (y_plus_short - measured[3]) ** 2
        )

    def _estimate_module(self, measured, prev):
        max_abs = abs(self.max_joint_abs_rad)

        def objective(x):
            a1 = self._clamp(float(x[0]), -max_abs, max_abs)
            a2 = self._clamp(float(x[1]), -max_abs, max_abs)
            fit = self._residual_sq(a1, a2, measured)
            smooth = self.w_prev * ((a1 - prev[0]) ** 2 + (a2 - prev[1]) ** 2)
            return fit + smooth

        x0 = [self._clamp(prev[0], -max_abs, max_abs), self._clamp(prev[1], -max_abs, max_abs)]
        res = minimize(
            objective,
            x0,
            method="L-BFGS-B",
            bounds=[(-max_abs, max_abs), (-max_abs, max_abs)],
            options={"maxiter": max(1, self.minimize_maxiter), "ftol": 1e-8},
        )
        return [float(res.x[0]), float(res.x[1])]

    def cb(self, msg):
        servo_map = {int(sv.index): int(sv.angle) for sv in msg.servos}

        required_indices = [idx for group in self.MODULE_SERVO_INDICES for idx in group]
        missing = [idx for idx in required_indices if idx not in servo_map]
        if missing:
            rospy.logwarn_throttle(1.0, "Missing servo indices in servo/states: %s", missing)
            return

        logical_joint_values = {}
        fitting_errors = []
        for module_i, (joint_3i_minus_1, joint_3i) in enumerate(self.MODULE_JOINT_PAIRS):
            i0, i1, i2, i3 = self.MODULE_SERVO_INDICES[module_i]

            # pub_soft_link_servo_states.py の並びに対応:
            # [y_plus_long, y_minus_long, y_minus_short, y_plus_short]
            measured = (
                angle_tick_to_wire_diff(self.servo_center - servo_map[i0]),
                angle_tick_to_wire_diff(self.servo_center - servo_map[i1]),
                angle_tick_to_wire_diff(self.servo_center - servo_map[i2]),
                angle_tick_to_wire_diff(self.servo_center - servo_map[i3]),
            )

            a1, a2 = self._estimate_module(measured, self.prev_est[module_i])
            self.prev_est[module_i] = [a1, a2]

            # forward側と同様に alpha_1 <- joint{3*i}, alpha_2 <- joint{3*i-1}
            logical_joint_values[joint_3i_minus_1] = a2
            logical_joint_values[joint_3i] = a1

            fitting_errors.append(math.sqrt(self._residual_sq(a1, a2, measured)))

        names = [j for pair in self.MODULE_JOINT_PAIRS for j in pair]
        out = JointState()
        out.header.stamp = rospy.Time.now()
        out.name = names
        out.position = [logical_joint_values[n] for n in names]
        self.pub.publish(out)

        rospy.loginfo_throttle(
            0.5,
            "Published estimated joint_states. module fit RMSE[mm-like]: %s",
            [round(e, 4) for e in fitting_errors],
        )


def main():
    try:
        EstimateJointStatesNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
