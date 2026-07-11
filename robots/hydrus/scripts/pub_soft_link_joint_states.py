#!/usr/bin/env python3

import math

import rospy
from scipy.optimize import minimize
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from fk import (
    DEFAULT_MODULE_PARAMS,
    DEFAULT_SOFT_L1,
    DEFAULT_SOFT_L2,
    DEFAULT_SOFT_L3,
    DEFAULT_SOFT_L4,
    DEFAULT_SOFT_L5,
    compute_end_pose_error_sq,
    mat_mul,
    rot,
    rotate_vec,
)


class PubSoftLinkJointStates(object):
    MODULE_JOINT_GROUPS = (
        ("soft_joint2", "soft_joint3", "soft_joint4", "soft_joint5"),
        ("soft_joint7", "soft_joint8", "soft_joint9", "soft_joint10"),
        ("soft_joint12", "soft_joint13", "soft_joint14", "soft_joint15"),
        ("soft_joint17", "soft_joint18", "soft_joint19", "soft_joint20"),
    )

    def __init__(self):
        rospy.init_node("pub_soft_link_joint_states")

        self.publish_hz = rospy.get_param("~publish_hz", 500.0)
        self.target_rotor13_distance = rospy.get_param("~rotor13_distance", 0.8)
        self.distance_topic = rospy.get_param("~rotor13_distance_topic", "target_rotor13_distance")
        self.max_joint_abs_rad = rospy.get_param("~max_joint_abs_rad", 1.2)
        self.ik_max_iters = int(rospy.get_param("~ik_max_iters", 120))
        self.ik_ftol = rospy.get_param("~ik_ftol", 1.0e-6)
        self.w_pose_pos = rospy.get_param("~ik_weight_pose_pos", 50.0)
        self.w_pose_yaw = rospy.get_param("~ik_weight_pose_yaw", 20.0)
        self.w_sum_360 = rospy.get_param("~ik_weight_sum_360", 120.0)
        self.w_rotor13 = rospy.get_param("~ik_weight_rotor13", 250.0)
        self.w_dash_sym = rospy.get_param("~ik_weight_dash_sym", 80.0)
        self.w_reg = rospy.get_param("~ik_weight_reg", 1.0)
        self.w_adjacent = rospy.get_param("~ik_weight_adjacent", 5.0)
        self.ik_restarts = int(rospy.get_param("~ik_restarts", 6))
        self.rotor_dash_ratio = max(0.0, min(1.0, rospy.get_param("~rotor_dash_ratio", 0.5)))
        self.logical_joint_names = [name for group in self.MODULE_JOINT_GROUPS for name in group]
        self.soft_joint_names = list(self.logical_joint_names)
        self._last_target_distance = None
        self._last_joint_pos = [math.radians(22.5)] * len(self.soft_joint_names)

        self.soft_l1 = rospy.get_param("~soft_l1", DEFAULT_SOFT_L1)
        self.soft_l2 = rospy.get_param("~soft_l2", DEFAULT_SOFT_L2)
        self.soft_l3 = rospy.get_param("~soft_l3", DEFAULT_SOFT_L3)
        self.soft_l4 = rospy.get_param("~soft_l4", DEFAULT_SOFT_L4)
        self.soft_l5 = rospy.get_param("~soft_l5", DEFAULT_SOFT_L5)
        self.rotor_offset_x = rospy.get_param("~rotor_offset_x", 0.0735)
        self.module_params = list(DEFAULT_MODULE_PARAMS)

        print("is_simulation: ", self.is_simulation())

        if self.is_simulation():
            self.joint_states_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
            self.joint_control_topic_name = "joints_ctrl"
        else:
            self.joint_states_pub = rospy.Publisher("dummy_joint_states", JointState, queue_size=1)
            self.joint_control_topic_name = "joint_states"
        self.target_soft_joint_pub = rospy.Publisher("target_soft_joints_ctrl", JointState, queue_size=1)
        self.distance_sub = rospy.Subscriber(self.distance_topic, Float64, self.distance_cb, queue_size=1)

        rospy.loginfo(
            "publish %s soft joints to %s at %.3f Hz (target rotor1-rotor3 distance: %.4f m)",
            len(self.soft_joint_names),
            self.joint_control_topic_name,
            self.publish_hz,
            self.target_rotor13_distance,
        )
        rospy.loginfo("subscribe target distance topic: %s", self.distance_topic)

    def distance_cb(self, msg):
        self.target_rotor13_distance = msg.data

    def run(self):
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown():
            self.target_rotor13_distance = rospy.get_param("~rotor13_distance", self.target_rotor13_distance)
            if (
                self._last_target_distance is None
                or abs(self.target_rotor13_distance - self._last_target_distance) > 1e-6
            ):
                self._last_joint_pos = self.compute_joint_positions(self.target_rotor13_distance)
                self._last_target_distance = self.target_rotor13_distance
            joint_pos = list(self._last_joint_pos)

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = list(self.soft_joint_names)
            msg.position = joint_pos
            self.joint_states_pub.publish(msg)
            self.target_soft_joint_pub.publish(msg)
            rate.sleep()

    def _yaw_from_rot(self, r):
        return math.atan2(r[1][0], r[0][0])

    @staticmethod
    def _wrap_to_pi(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def forward_kinematics(self, joints):
        p = (0.0, 0.0)
        r = ((1.0, 0.0), (0.0, 1.0))
        theta_raw = 0.0
        rotor_ps = []
        rotor_dash_ps = []
        joint_pos_map = {}
        soft_lengths = [self.soft_l1, self.soft_l2, self.soft_l3, self.soft_l4, self.soft_l5]

        for module_i, module in enumerate(self.module_params):
            module_offset = rotate_vec(r, (module["parent_to_servo_x"], 0.0))
            p = (p[0] + module_offset[0], p[1] + module_offset[1])

            module_joints = joints[4 * module_i : 4 * module_i + 4]
            module_joint_names = self.MODULE_JOINT_GROUPS[module_i]
            for link_i, link_len in enumerate(soft_lengths):
                link = rotate_vec(r, (link_len, 0.0))
                p = (p[0] + link[0], p[1] + link[1])
                if link_i < len(module_joints):
                    q = module_joints[link_i]
                    joint_pos_map[module_joint_names[link_i]] = (p[0], p[1])
                    theta_raw += q
                    r = mat_mul(r, rot(q))

            tail = rotate_vec(r, (module["servo_size_x"], 0.0))
            p = (p[0] + tail[0], p[1] + tail[1])

            rotor_offset = rotate_vec(r, (self.rotor_offset_x, 0.0))
            rotor = (p[0] + rotor_offset[0], p[1] + rotor_offset[1])
            rotor_ps.append(rotor)
            rotor_dash_ps.append(
                (
                    p[0] + self.rotor_dash_ratio * rotor_offset[0],
                    p[1] + self.rotor_dash_ratio * rotor_offset[1],
                )
            )

        return {
            "end_pose": (p[0], p[1], self._yaw_from_rot(r), theta_raw),
            "rotors": rotor_ps,
            "rotor_dash": rotor_dash_ps,
            "joint_pos": joint_pos_map,
        }

    def ik_cost(self, joints, target_distance, ref_joints):
        fk = self.forward_kinematics(joints)
        end_x, end_y, end_yaw, _ = fk["end_pose"]
        rotor1_dash = fk["rotor_dash"][0]
        rotor3_dash = fk["rotor_dash"][2]
        rotor13 = math.hypot(rotor3_dash[0] - rotor1_dash[0], rotor3_dash[1] - rotor1_dash[1])

        module4_first = fk["joint_pos"]["soft_joint17"]
        module1_last = fk["joint_pos"]["soft_joint5"]
        d1 = math.hypot(rotor1_dash[0] - module4_first[0], rotor1_dash[1] - module4_first[1])
        d3 = math.hypot(rotor3_dash[0] - module1_last[0], rotor3_dash[1] - module1_last[1])
        dash_sym_err2 = (d1 - d3) ** 2

        pose_pos_err2, sum_360_err2 = compute_end_pose_error_sq(
            joints,
            module_params=self.module_params,
            soft_l1=self.soft_l1,
            soft_l2=self.soft_l2,
            soft_l3=self.soft_l3,
            soft_l4=self.soft_l4,
            soft_l5=self.soft_l5,
        )
        pose_yaw_err2 = self._wrap_to_pi(end_yaw) ** 2
        rotor13_err2 = (rotor13 - target_distance) ** 2
        adjacent_err2 = 0.0
        for module_i in range(len(self.MODULE_JOINT_GROUPS)):
            off = 4 * module_i
            adjacent_err2 += (
                (joints[off] - joints[off + 1]) ** 2
                + (joints[off + 1] - joints[off + 2]) ** 2
                + (joints[off + 2] - joints[off + 3]) ** 2
            )
        reg = sum((q - q_ref) ** 2 for q, q_ref in zip(joints, ref_joints))

        return (
            self.w_pose_pos * pose_pos_err2
            + self.w_pose_yaw * pose_yaw_err2
            + self.w_sum_360 * sum_360_err2
            + self.w_rotor13 * rotor13_err2
            + self.w_dash_sym * dash_sym_err2
            + self.w_adjacent * adjacent_err2
            + self.w_reg * reg
        )

    def solve_ik(self, target_distance):
        q_ref = [math.radians(22.5)] * len(self.logical_joint_names)
        max_abs = abs(self.max_joint_abs_rad)
        seeds = [list(q_ref), [0.0] * len(q_ref), [-v for v in q_ref]]
        seeds = seeds[: max(1, min(len(seeds), 1 + self.ik_restarts))]
        bounds = [(-max_abs, max_abs)] * len(self.logical_joint_names)

        def objective(q):
            return self.ik_cost(q, target_distance, q_ref)

        best_q = list(q_ref)
        best_cost = self.ik_cost(best_q, target_distance, q_ref)

        for seed in seeds:
            x0 = [max(-max_abs, min(max_abs, v)) for v in seed]
            res = minimize(
                objective,
                x0,
                method="SLSQP",
                bounds=bounds,
                options={"maxiter": max(1, self.ik_max_iters), "ftol": self.ik_ftol, "disp": False},
            )
            if hasattr(res, "x"):
                cand_q = [float(v) for v in res.x]
            else:
                cand_q = list(x0)
            cand_cost = self.ik_cost(cand_q, target_distance, q_ref)

            if cand_cost < best_cost:
                best_cost = cand_cost
                best_q = cand_q

        return best_q

    def compute_joint_positions(self, target_distance):
        q = self.solve_ik(target_distance)
        fk = self.forward_kinematics(q)
        rotor1 = fk["rotor_dash"][0]
        rotor3 = fk["rotor_dash"][2]
        actual_d = math.hypot(rotor3[0] - rotor1[0], rotor3[1] - rotor1[1])
        end_x, end_y, end_yaw, end_theta_raw = fk["end_pose"]
        print(
            "IK solved: target d13_dash={0:.4f}, actual d13_dash={1:.4f}, end=({2:.4f}, {3:.4f}, yaw={4:.3f}deg, sum={5:.3f}deg)".format(
                target_distance, actual_d, end_x, end_y, math.degrees(end_yaw), math.degrees(end_theta_raw)
            )
        )
        return q

    def is_simulation(self):
        return rospy.get_param("/use_sim_time", False)


if __name__ == "__main__":
    node = PubSoftLinkJointStates()
    node.run()
