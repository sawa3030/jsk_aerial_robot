#!/usr/bin/env python

import argparse
import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.optimize import minimize


class SoftLinkIKVisualizer(object):
    # Logical (optimization) joints:
    # module1: soft_joint2,  soft_joint3
    # module2: soft_joint5,  soft_joint6
    # module3: soft_joint8,  soft_joint9
    # module4: soft_joint11, soft_joint12
    LOGICAL_JOINT_ORDER = (
        "soft_joint2", "soft_joint3", "soft_joint5", "soft_joint6",
        "soft_joint8", "soft_joint9", "soft_joint11", "soft_joint12",
    )
    # Physical (visualized/output) joints:
    # each logical joint is duplicated and constrained to bend identically.
    JOINT_ORDER = (
        "soft_joint2_a", "soft_joint2_b", "soft_joint3_a", "soft_joint3_b",
        "soft_joint5_a", "soft_joint5_b", "soft_joint6_a", "soft_joint6_b",
        "soft_joint8_a", "soft_joint8_b", "soft_joint9_a", "soft_joint9_b",
        "soft_joint11_a", "soft_joint11_b", "soft_joint12_a", "soft_joint12_b",
    )

    def __init__(self, args):
        self.max_joint_abs_rad = args.max_joint_abs_rad
        self.ik_max_iters = int(args.ik_max_iters)
        self.ik_ftol = args.ik_ftol
        self.w_pose_pos = args.ik_weight_pose_pos
        self.w_pose_yaw = args.ik_weight_pose_yaw
        self.w_sum_360 = args.ik_weight_sum_360
        self.w_rotor13 = args.ik_weight_rotor13
        self.w_dash_sym = args.ik_weight_dash_sym
        self.w_reg = args.ik_weight_reg
        self.ik_restarts = int(args.ik_restarts)
        self.rotor_dash_ratio = max(0.0, min(1.0, args.rotor_dash_ratio))

        self.logical_joint_names = list(self.LOGICAL_JOINT_ORDER)
        self.soft_joint_names = list(self.JOINT_ORDER)

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
    def _duplicate_joint_pairs(logical_joints):
        physical = []
        for q in logical_joints:
            physical.append(q)
            physical.append(q)
        return physical

    def forward_kinematics(self, logical_joints):
        # logical joints order: [s2,s3,s5,s6,s8,s9,s11,s12]
        # physical joints order: [s2a,s2b,s3a,s3b,...], where each pair is identical.
        joints = self._duplicate_joint_pairs(logical_joints)
        p = (0.0, 0.0)
        r = ((1.0, 0.0), (0.0, 1.0))
        theta_raw = 0.0
        gimbal_ps = []
        rotor_ps = []
        rotor_dash_ps = []
        chain_ps = [p]
        joint_labels = []
        joint_pos_map = {}

        for i in range(4):
            q1a = joints[4 * i]
            q1b = joints[4 * i + 1]
            q2a = joints[4 * i + 2]
            q2b = joints[4 * i + 3]
            name1a = self.soft_joint_names[4 * i]
            name1b = self.soft_joint_names[4 * i + 1]
            name2a = self.soft_joint_names[4 * i + 2]
            name2b = self.soft_joint_names[4 * i + 3]
            m = self.module_params[i]

            # module start offset
            module_offset = self._rotate_vec(r, (m["parent_to_servo_x"], 0.0))
            p = (p[0] + module_offset[0], p[1] + module_offset[1])
            chain_ps.append(p)

            # Split soft part into 6 equal segments with 4 joints in between:
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
                chain_ps.append(p)
                if group_i < len(module_joints):
                    q = module_joints[group_i]
                    joint_labels.append((module_joint_names[group_i], p[0], p[1]))
                    joint_pos_map[module_joint_names[group_i]] = (p[0], p[1])
                    theta_raw += q
                    r = self._mat_mul(r, self._rot(q))

            # tail rigid for servo body
            tail = self._rotate_vec(r, (m["servo_size_x"], 0.0))
            p = (p[0] + tail[0], p[1] + tail[1])
            chain_ps.append(p)

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
            "chain": chain_ps,
            "joint_labels": joint_labels,
            "joint_pos": joint_pos_map,
        }

    def ik_cost(self, joints, target_distance):
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
        print(dash_sym_err2)

        pose_pos_err2 = end_x * end_x + end_y * end_y
        pose_yaw_err2 = self._wrap_to_pi(end_yaw) ** 2
        sum_360_err2 = (end_theta_raw - 2.0 * math.pi) ** 2
        rotor13_err2 = (rotor13 - target_distance) ** 2
        reg = sum(q * q for q in joints)
        return (
            self.w_pose_pos * pose_pos_err2
            # + self.w_pose_yaw * pose_yaw_err2
            + self.w_sum_360 * sum_360_err2
            + self.w_rotor13 * rotor13_err2
            + self.w_dash_sym * dash_sym_err2
            + self.w_reg * reg
        )

    def solve_ik(self, target_distance):
        q_ref = [0.25 * math.pi] * len(self.logical_joint_names)
        max_abs = abs(self.max_joint_abs_rad)
        seeds = [list(q_ref)]
        n_seeds = max(1, min(len(seeds), 1 + self.ik_restarts))
        seeds = seeds[:n_seeds]
        bounds = [(-max_abs, max_abs)] * len(self.logical_joint_names)

        def objective(q):
            return self.ik_cost(q, target_distance)

        best_q = list(q_ref)
        best_cost = self.ik_cost(best_q, target_distance)

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
            cand_cost = self.ik_cost(cand_q, target_distance)

            if cand_cost < best_cost:
                best_cost = cand_cost
                best_q = cand_q

        return best_q

    def solve_once(self, target_distance):
        logical_q = self.solve_ik(target_distance)
        q = self._duplicate_joint_pairs(logical_q)
        fk = self.forward_kinematics(logical_q)
        rotor1 = fk["rotor_dash"][0]
        rotor3 = fk["rotor_dash"][2]
        actual_d = math.hypot(rotor3[0] - rotor1[0], rotor3[1] - rotor1[1])
        return q, fk, actual_d


def _print_result(joint_names, joints, target_distance, fk, actual_d):
    print("target rotor1_dash-rotor3_dash distance: {:.4f} [m]".format(target_distance))
    print("actual rotor1_dash-rotor3_dash distance: {:.4f} [m]".format(actual_d))
    end_x, end_y, end_yaw, end_theta_raw = fk["end_pose"]
    print(
        "end pose: x={:.4f}, y={:.4f}, yaw={:.3f}[deg], theta_sum={:.3f}[deg]".format(
            end_x, end_y, math.degrees(end_yaw), math.degrees(end_theta_raw)
        )
    )
    print("joint angles [rad]:")
    for name, q in zip(joint_names, joints):
        print("  {:>12s}: {:+.5f}".format(name, q))


def _plot_state(ax, fk, target_distance, actual_distance):
    chain = fk["chain"]
    rotors = fk["rotors"]
    rotor_dash = fk.get("rotor_dash", [])
    gimbals = fk["gimbals"]
    joint_labels = fk.get("joint_labels", [])

    xs = [p[0] for p in chain]
    ys = [p[1] for p in chain]
    rx = [p[0] for p in rotors]
    ry = [p[1] for p in rotors]
    rdx = [p[0] for p in rotor_dash]
    rdy = [p[1] for p in rotor_dash]
    gx = [p[0] for p in gimbals]
    gy = [p[1] for p in gimbals]

    ax.clear()
    ax.plot(xs, ys, "-o", linewidth=2.0, markersize=3.0, color="#1f77b4", label="soft-link chain")
    ax.scatter(gx, gy, s=45, c="#2ca02c", label="gimbal")
    ax.scatter(rx, ry, s=75, marker="x", c="#d62728", label="rotor")
    if rotor_dash:
        ax.scatter(rdx, rdy, s=55, marker="D", c="#9467bd", label="rotor_dash")
        for i in range(min(len(rotors), len(rotor_dash))):
            ax.plot(
                [rotor_dash[i][0], rotors[i][0]],
                [rotor_dash[i][1], rotors[i][1]],
                linestyle=":",
                linewidth=1.0,
                color="#9467bd",
            )
    for name, jx, jy in joint_labels:
        ax.text(jx, jy, name, fontsize=7, color="#444444", ha="left", va="bottom")

    if len(rotor_dash) >= 3:
        r1 = rotor_dash[0]
        r3 = rotor_dash[2]
        d13_label = "d13_dash"
    else:
        r1 = rotors[0]
        r3 = rotors[2]
        d13_label = "d13"
    ax.plot([r1[0], r3[0]], [r1[1], r3[1]], "--", color="#ff7f0e", linewidth=1.8, label=d13_label)

    ax.text(
        0.02,
        0.98,
        "target d13_dash={:.4f} m\nactual d13_dash={:.4f} m".format(target_distance, actual_distance),
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
    )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Hydrus Soft-Link IK (No ROS)")
    ax.axis("equal")
    ax.grid(True, linestyle=":", linewidth=0.7)
    ax.legend(loc="best")


def _visualize_static(solver, target_distance):
    joints, fk, actual_d = solver.solve_once(target_distance)
    _print_result(solver.soft_joint_names, joints, target_distance, fk, actual_d)

    fig, ax = plt.subplots(figsize=(7.5, 7.0))
    _plot_state(ax, fk, target_distance, actual_d)
    plt.tight_layout()
    plt.show()


def _visualize_animation(solver, d_min, d_max, steps, interval_ms):
    distances = []
    if steps <= 1:
        distances = [d_min]
    else:
        step = (d_max - d_min) / float(steps - 1)
        distances = [d_min + i * step for i in range(steps)]

    forward = list(distances)
    backward = list(reversed(distances[1:-1]))
    cycle = forward + backward if len(forward) > 1 else forward

    fig, ax = plt.subplots(figsize=(7.5, 7.0))

    def update(i):
        target_distance = cycle[i % len(cycle)]
        _, fk, actual_d = solver.solve_once(target_distance)
        _plot_state(ax, fk, target_distance, actual_d)
        _, _, end_yaw, end_theta_raw = fk["end_pose"]
        ax.set_title(
            "Hydrus Soft-Link IK (No ROS)\n"
            "frame={} yaw={:.2f}deg theta_sum={:.2f}deg".format(
                i, math.degrees(end_yaw), math.degrees(end_theta_raw)
            )
        )
        return []

    FuncAnimation(fig, update, interval=interval_ms, blit=False)
    plt.tight_layout()
    plt.show()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Solve Hydrus soft-link IK without ROS and visualize the result."
    )
    parser.add_argument("--target-distance", type=float, default=0.95,
                        help="target rotor1_dash-rotor3_dash distance [m] for static visualization")
    parser.add_argument("--animate", action="store_true",
                        help="animate by sweeping target distance")
    parser.add_argument("--distance-min", type=float, default=0.75,
                        help="minimum target distance [m] for animation")
    parser.add_argument("--distance-max", type=float, default=1.05,
                        help="maximum target distance [m] for animation")
    parser.add_argument("--steps", type=int, default=40,
                        help="distance samples in one forward sweep")
    parser.add_argument("--interval-ms", type=int, default=120,
                        help="animation interval [ms]")

    parser.add_argument("--max-joint-abs-rad", type=float, default=1.2)
    parser.add_argument("--ik-max-iters", type=int, default=120)
    parser.add_argument("--ik-ftol", type=float, default=1.0e-6)
    parser.add_argument("--ik-weight-pose-pos", type=float, default=50.0)
    parser.add_argument("--ik-weight-pose-yaw", type=float, default=20.0)
    parser.add_argument("--ik-weight-sum-360", type=float, default=120.0)
    parser.add_argument("--ik-weight-rotor13", type=float, default=250.0)
    parser.add_argument("--ik-weight-dash-sym", type=float, default=80.0)
    parser.add_argument("--ik-weight-reg", type=float, default=1.0)
    parser.add_argument("--ik-weight-smooth", type=float, default=0.2)
    parser.add_argument("--ik-weight-mirror-sym", type=float, default=10.0)
    parser.add_argument("--ik-weight-ccw-order", type=float, default=200.0)
    parser.add_argument("--ik-ccw-min-sep-deg", type=float, default=20.0)
    parser.add_argument("--ik-restarts", type=int, default=6)
    parser.add_argument("--rotor-dash-ratio", type=float, default=0.5,
                        help="0.0=gimbal, 1.0=rotor; rotor_dash point ratio on gimbal-rotor link")

    return parser.parse_args()


def main():
    args = parse_args()
    solver = SoftLinkIKVisualizer(args)

    if args.animate:
        _visualize_animation(
            solver,
            d_min=min(args.distance_min, args.distance_max),
            d_max=max(args.distance_min, args.distance_max),
            steps=max(1, args.steps),
            interval_ms=max(1, args.interval_ms),
        )
    else:
        _visualize_static(solver, args.target_distance)


if __name__ == "__main__":
    main()
