#!/usr/bin/env python

import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from scipy.optimize import minimize


class PubSoftLinkJointStates(object):
    # soft_airframe_202605 actuated joints (logical joints):
    # module1: soft_joint2,  soft_joint3
    # module2: soft_joint5,  soft_joint6
    # module3: soft_joint8,  soft_joint9
    # module4: soft_joint11, soft_joint12
    LOGICAL_JOINT_ORDER = ("soft_joint2", "soft_joint3", "soft_joint5", "soft_joint6",
                           "soft_joint8", "soft_joint9", "soft_joint11", "soft_joint12")
    # Virtual joints used by the no-ROS IK logic:
    # each logical joint is duplicated and constrained to bend identically.
    VIRTUAL_JOINT_ORDER = (
        "soft_joint2_a", "soft_joint2_b", "soft_joint3_a", "soft_joint3_b",
        "soft_joint5_a", "soft_joint5_b", "soft_joint6_a", "soft_joint6_b",
        "soft_joint8_a", "soft_joint8_b", "soft_joint9_a", "soft_joint9_b",
        "soft_joint11_a", "soft_joint11_b", "soft_joint12_a", "soft_joint12_b",
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
        self.ik_restarts = int(rospy.get_param("~ik_restarts", 6))
        self.rotor_dash_ratio = max(0.0, min(1.0, rospy.get_param("~rotor_dash_ratio", 0.5)))
        self.logical_joint_names = list(self.LOGICAL_JOINT_ORDER)
        self.virtual_joint_names = list(self.VIRTUAL_JOINT_ORDER)
        self.soft_joint_names = list(self.logical_joint_names)
        self._last_target_distance = None
        self._last_joint_pos = [0.0] * len(self.soft_joint_names)

        # Geometry from hydrus/urdf/soft_link.urdf.xacro
        self.soft_l1 = 0.1175
        self.soft_l2 = 0.235
        self.soft_l3 = 0.1175
        self.rotor_offset_x = 0.0735
        self.module_params = [
            {"parent_to_servo_x": 0.0, "servo_size_x": 0.096},   # module1
            {"parent_to_servo_x": 0.147, "servo_size_x": 0.156}, # module2
            {"parent_to_servo_x": 0.147, "servo_size_x": 0.096}, # module3
            {"parent_to_servo_x": 0.147, "servo_size_x": 0.096}, # module4
        ]
        self.soft_segment_divisions = 6

        print("is_simulation: ", self.is_simulation())

        if self.is_simulation():
            self.joint_states_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
            self.joint_control_topic_name = "joints_ctrl"
        else:
            self.joint_states_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
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
            if (self._last_target_distance is None or
                    abs(self.target_rotor13_distance - self._last_target_distance) > 1e-6):
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

    @staticmethod
    def _rot(theta):
        c = math.cos(theta)
        s = math.sin(theta)
        return ((c, -s), (s, c))

    @staticmethod
    def _mat_mul(a, b):
        return (
            (a[0][0] * b[0][0] + a[0][1] * b[1][0], a[0][0] * b[0][1] + a[0][1] * b[1][1]),
            (a[1][0] * b[0][0] + a[1][1] * b[1][0], a[1][0] * b[0][1] + a[1][1] * b[1][1]),
        )

    @staticmethod
    def _rotate_vec(r, v):
        return (
            r[0][0] * v[0] + r[0][1] * v[1],
            r[1][0] * v[0] + r[1][1] * v[1],
        )

    def _yaw_from_rot(self, r):
        return math.atan2(r[1][0], r[0][0])

    @staticmethod
    def _wrap_to_pi(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _wrap_to_2pi(angle):
        while angle >= 2.0 * math.pi:
            angle -= 2.0 * math.pi
        while angle < 0.0:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _duplicate_joint_pairs(logical_joints):
        physical = []
        for q in logical_joints:
            physical.append(q)
            physical.append(q)
        return physical

    def forward_kinematics(self, logical_joints):
        # logical joints order: [s2,s3,s5,s6,s8,s9,s11,s12]
        # virtual joints order: [s2a,s2b,s3a,s3b,...], where each pair is identical.
        joints = self._duplicate_joint_pairs(logical_joints)
        p = (0.0, 0.0)
        r = ((1.0, 0.0), (0.0, 1.0))
        theta_raw = 0.0
        gimbal_ps = []
        rotor_ps = []
        rotor_dash_ps = []
        joint_pos_map = {}

        for i in range(4):
            q1a = joints[4 * i]
            q1b = joints[4 * i + 1]
            q2a = joints[4 * i + 2]
            q2b = joints[4 * i + 3]
            name1a = self.virtual_joint_names[4 * i]
            name1b = self.virtual_joint_names[4 * i + 1]
            name2a = self.virtual_joint_names[4 * i + 2]
            name2b = self.virtual_joint_names[4 * i + 3]
            m = self.module_params[i]

            # module start offset
            module_offset = self._rotate_vec(r, (m["parent_to_servo_x"], 0.0))
            p = (p[0] + module_offset[0], p[1] + module_offset[1])

            # Split soft part into 6 equal segments with 4 virtual joints in between:
            # segment1-(joint1), segment2-(joint2), segment3+4-(joint3),
            # segment5-(joint4), segment6.
            soft_total = self.soft_l1 + self.soft_l2 + self.soft_l3
            soft_seg = soft_total / float(self.soft_segment_divisions)
            module_joints = [q1a, q1b, q2a, q2b]
            module_joint_names = [name1a, name1b, name2a, name2b]
            seg_groups = [1, 1, 2, 1, 1]
            for group_i, n_segs in enumerate(seg_groups):
                link = self._rotate_vec(r, (soft_seg * float(n_segs), 0.0))
                p = (p[0] + link[0], p[1] + link[1])
                if group_i < len(module_joints):
                    q = module_joints[group_i]
                    joint_pos_map[module_joint_names[group_i]] = (p[0], p[1])
                    theta_raw += q
                    r = self._mat_mul(r, self._rot(q))

            # tail rigid for servo body
            tail = self._rotate_vec(r, (m["servo_size_x"], 0.0))
            p = (p[0] + tail[0], p[1] + tail[1])

            gimbal_ps.append((p[0], p[1], self._yaw_from_rot(r)))
            rotor_offset = self._rotate_vec(r, (self.rotor_offset_x, 0.0))
            rotor = (p[0] + rotor_offset[0], p[1] + rotor_offset[1])
            rotor_ps.append(rotor)
            # rotor_dash is constrained on the rigid link between gimbal and rotor.
            rotor_dash_ps.append((
                p[0] + self.rotor_dash_ratio * rotor_offset[0],
                p[1] + self.rotor_dash_ratio * rotor_offset[1],
            ))

        return {
            "end_pose": (p[0], p[1], self._yaw_from_rot(r), theta_raw),
            "gimbals": gimbal_ps,
            "rotors": rotor_ps,
            "rotor_dash": rotor_dash_ps,
            "joint_pos": joint_pos_map,
        }

    def ik_cost(self, joints, target_distance, ref_joints):
        fk = self.forward_kinematics(joints)
        end_x, end_y, end_yaw, end_theta_raw = fk["end_pose"]
        rotor1 = fk["rotor_dash"][0]
        rotor3 = fk["rotor_dash"][2]
        rotor13 = math.hypot(rotor3[0] - rotor1[0], rotor3[1] - rotor1[1])

        rotor1_dash = fk["rotor_dash"][0]
        rotor3_dash = fk["rotor_dash"][2]
        joint11a = fk["joint_pos"]["soft_joint11_a"]
        joint3b = fk["joint_pos"]["soft_joint3_b"]
        d1 = math.hypot(rotor1_dash[0] - joint11a[0], rotor1_dash[1] - joint11a[1])
        d3 = math.hypot(rotor3_dash[0] - joint3b[0], rotor3_dash[1] - joint3b[1])
        dash_sym_err2 = (d1 - d3) ** 2

        pose_pos_err2 = end_x * end_x + end_y * end_y
        pose_yaw_err2 = self._wrap_to_pi(end_yaw) ** 2
        sum_360_err2 = (end_theta_raw - 2.0 * math.pi) ** 2
        rotor13_err2 = (rotor13 - target_distance) ** 2
        reg = sum(q * q for q in joints)
        _ = ref_joints

        return (
            self.w_pose_pos * pose_pos_err2
            # + self.w_pose_yaw * pose_yaw_err2
            + self.w_sum_360 * sum_360_err2
            + self.w_rotor13 * rotor13_err2
            + self.w_dash_sym * dash_sym_err2
            + self.w_reg * reg
        )

    def solve_ik(self, target_distance):
        # Always start IK from 45deg for each logical joint (do not reuse previous solution).
        q_ref = [0.25 * math.pi] * len(self.logical_joint_names)
        max_abs = abs(self.max_joint_abs_rad)
        seeds = [list(q_ref)]
        n_seeds = max(1, min(len(seeds), 1 + self.ik_restarts))
        seeds = seeds[:n_seeds]
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
            if res.success:
                cand_q = [float(v) for v in res.x]
                cand_cost = float(res.fun)
            else:
                # fallback: evaluate final iterate even if not fully converged
                cand_q = [float(v) for v in res.x] if hasattr(res, "x") else list(x0)
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
        # rospy.loginfo_throttle(
        #     1.0,
        #     "IK solved: target d13_dash=%.4f, actual d13_dash=%.4f, end=(%.4f, %.4f, yaw=%.3fdeg, sum=%.3fdeg)",
        #     target_distance, actual_d, end_x, end_y, math.degrees(end_yaw), math.degrees(end_theta_raw),
        # )
        print(
            "IK solved: target d13_dash={0:.4f}, actual d13_dash={1:.4f}, end=({2:.4f}, {3:.4f}, yaw={4:.3f}deg, sum={5:.3f}deg)".format(
                target_distance, actual_d, end_x, end_y, math.degrees(end_yaw), math.degrees(end_theta_raw)
            )
        )
        q = [v * 2.0 for v in q]
        return q

    def is_simulation(self) -> bool:
        return rospy.get_param('/use_sim_time', False)


if __name__ == "__main__":
    node = PubSoftLinkJointStates()
    node.run()
