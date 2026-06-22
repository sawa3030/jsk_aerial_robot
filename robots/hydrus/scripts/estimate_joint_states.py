#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from scipy.optimize import minimize
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates

from fk import compute_end_pose_error_sq

# ---- Tail wire model params (pub_soft_link_servo_states.py と同じ) ----
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
            get_plus_pos_wire_length(a1, r_joint_2, divide_num)
            + d
            + get_plus_pos_wire_length(a2, r_joint_2, divide_num)
        )
        minus_short_wire = (
            get_minus_pos_wire_length(a1, r_joint_2, divide_num)
            + d
            + get_minus_pos_wire_length(a2, r_joint_2, divide_num)
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


def angle_tick_to_wire_diff(tick_diff):
    return float(tick_diff) * math.pi * r_wheel / 4096.0


class EstimateJointStatesNode:
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
        self.w_pose_pos = float(rospy.get_param("~weight_pose_pos", 500.0))
        self.w_sum_360 = float(rospy.get_param("~weight_sum_360", 120.0))
        self.w_adjacent = float(rospy.get_param("~weight_adjacent", 5))
        self.minimize_maxiter = int(rospy.get_param("~ik_max_iters", 120))
        servo_topic = rospy.get_param("~servo_states_topic", "servo/states")
        joint_topic = rospy.get_param("~joint_states_topic", "joint_states")

        self.pub = rospy.Publisher(joint_topic, JointState, queue_size=1)
        self.sub = rospy.Subscriber(servo_topic, ServoStates, self.cb, queue_size=1)

        q0 = math.radians(22.5)
        self.prev_est = [q0] * (len(self.MODULE_JOINT_GROUPS) * 4)

        rospy.loginfo(
            "estimate_joint_states started. sub=%s, pub=%s",
            servo_topic,
            joint_topic,
        )

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _residual_sq(a1, a2, a3, a4, measured):
        y_plus_long, y_minus_long, y_plus_short, y_minus_short = get_wire_diff(a1, a2, a3, a4)
        return (
            (y_plus_long - measured[0]) ** 2
            + (y_minus_long - measured[1]) ** 2
            + (y_minus_short - measured[2]) ** 2
            + (y_plus_short - measured[3]) ** 2
        )

    def _compute_cost_terms(self, q, measured_by_module, prev):
        n_joints = len(self.MODULE_JOINT_GROUPS) * 4
        fit = 0.0
        adjacent = 0.0
        for module_i, measured in enumerate(measured_by_module):
            off = 4 * module_i
            fit += self._residual_sq(q[off], q[off + 1], q[off + 2], q[off + 3], measured)
            adjacent += (
                (q[off] - q[off + 1]) ** 2
                + (q[off + 1] - q[off + 2]) ** 2
                + (q[off + 2] - q[off + 3]) ** 2
            )

        smooth = self.w_prev * sum((q[i] - prev[i]) ** 2 for i in range(n_joints))
        pose_pos_err2, sum_360_err2 = compute_end_pose_error_sq(q)
        closure = self.w_pose_pos * pose_pos_err2 + self.w_sum_360 * sum_360_err2
        adjacent_cost = self.w_adjacent * adjacent
        return fit, closure, smooth, adjacent_cost, pose_pos_err2, sum_360_err2

    def _estimate_all_modules(self, measured_by_module, prev):
        max_abs = abs(self.max_joint_abs_rad)
        n_joints = len(self.MODULE_JOINT_GROUPS) * 4

        def objective(x):
            q = [self._clamp(float(x[i]), -max_abs, max_abs) for i in range(n_joints)]
            fit, closure, smooth, adjacent_cost, _, _ = self._compute_cost_terms(
                q, measured_by_module, prev
            )
            return fit + smooth + closure + adjacent_cost

        x0 = [self._clamp(v, -max_abs, max_abs) for v in prev]
        res = minimize(
            objective,
            x0,
            method="L-BFGS-B",
            bounds=[(-max_abs, max_abs)] * n_joints,
            options={"maxiter": max(1, self.minimize_maxiter), "ftol": 1e-8},
        )
        return [float(v) for v in res.x]

    def cb(self, msg):
        servo_map = {int(sv.index): int(sv.angle) for sv in msg.servos}

        required_indices = [idx for group in self.MODULE_SERVO_INDICES for idx in group]
        missing = [idx for idx in required_indices if idx not in servo_map]
        if missing:
            rospy.logwarn_throttle(1.0, "Missing servo indices in servo/states: %s", missing)
            return

        measured_by_module = []
        for module_i, joint_group in enumerate(self.MODULE_JOINT_GROUPS):
            i0, i1, i2, i3 = self.MODULE_SERVO_INDICES[module_i]

            # pub_soft_link_servo_states.py の並びと一致:
            # [y_plus_long, y_minus_long, y_minus_short, y_plus_short]
            measured = (
                angle_tick_to_wire_diff(self.servo_center - servo_map[i0]),
                angle_tick_to_wire_diff(self.servo_center - servo_map[i1]),
                angle_tick_to_wire_diff(self.servo_center - servo_map[i2]),
                angle_tick_to_wire_diff(self.servo_center - servo_map[i3]),
            )
            measured_by_module.append(measured)

        prev_est = list(self.prev_est)
        est_all = self._estimate_all_modules(measured_by_module, prev_est)
        self.prev_est = list(est_all)

        logical_joint_values = {}
        fitting_errors = []
        for module_i, joint_group in enumerate(self.MODULE_JOINT_GROUPS):
            off = 4 * module_i
            est = est_all[off : off + 4]
            for joint_name, q in zip(joint_group, est):
                logical_joint_values[joint_name] = q
            measured = measured_by_module[module_i]
            fitting_errors.append(math.sqrt(self._residual_sq(est[0], est[1], est[2], est[3], measured)))

        fit, closure, smooth, adjacent_cost, pose_pos_err2, sum_360_err2 = self._compute_cost_terms(
            est_all, measured_by_module, prev_est
        )

        names = [j for group in self.MODULE_JOINT_GROUPS for j in group]

        out = JointState()
        out.header.stamp = rospy.Time.now()
        out.name = names
        out.position = [logical_joint_values[n] for n in names]
        self.pub.publish(out)

        rospy.loginfo_throttle(
            0.5,
            "Published estimated joint_states. module fit RMSE[mm-like]: %s, pose_pos_err2=%.6f, sum_360_err2=%.6f, cost(fit=%.6f, close=%.6f, smooth=%.6f, adjacent=%.6f)",
            [round(e, 4) for e in fitting_errors],
            pose_pos_err2,
            sum_360_err2,
            fit,
            closure,
            smooth,
            adjacent_cost,
        )


def main():
    try:
        EstimateJointStatesNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
