#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import rosbag

from estimate_joint_states import MODULE_JOINT_GROUPS, estimate_all_modules_from_servo_map
from fk import DEFAULT_MODULE_PARAMS, DEFAULT_ROTOR_OFFSET_X
from fk import DEFAULT_SOFT_L1, DEFAULT_SOFT_L2, DEFAULT_SOFT_L3, DEFAULT_SOFT_L4, DEFAULT_SOFT_L5

PHYSICAL_MODULE_JOINT_GROUPS = (
    ("soft_joint2", "soft_joint3", "soft_joint4", "soft_joint5"),
    ("soft_joint7", "soft_joint8", "soft_joint9", "soft_joint10"),
    ("soft_joint12", "soft_joint13", "soft_joint14", "soft_joint15"),
    ("soft_joint17", "soft_joint18", "soft_joint19", "soft_joint20"),
)
DEFAULT_FIGSIZE = (8.0, 7.0)
DEFAULT_XLIM = (-1.0, 1.0)
DEFAULT_YLIM = (-0.2, 1.5)


def rot(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return ((c, -s), (s, c))


def mat_mul(a, b):
    return (
        (a[0][0] * b[0][0] + a[0][1] * b[1][0], a[0][0] * b[0][1] + a[0][1] * b[1][1]),
        (a[1][0] * b[0][0] + a[1][1] * b[1][0], a[1][0] * b[0][1] + a[1][1] * b[1][1]),
    )


def rotate_vec(r, v):
    return (
        r[0][0] * v[0] + r[0][1] * v[1],
        r[1][0] * v[0] + r[1][1] * v[1],
    )


def compute_planar_shape(
    joints,
    joint_count=None,
    module_params=None,
    soft_l1=DEFAULT_SOFT_L1,
    soft_l2=DEFAULT_SOFT_L2,
    soft_l3=DEFAULT_SOFT_L3,
    soft_l4=DEFAULT_SOFT_L4,
    soft_l5=DEFAULT_SOFT_L5,
    rotor_offset_x=DEFAULT_ROTOR_OFFSET_X,
):
    if module_params is None:
        module_params = DEFAULT_MODULE_PARAMS
    if joint_count is None:
        joint_count = len(joints)

    if joint_count == 8:
        if len(joints) != 8:
            raise ValueError("Expected 8 joints, got {}".format(len(joints)))
        soft_total = soft_l1 + soft_l2 + soft_l3 + soft_l4 + soft_l5
        soft_lengths = (soft_total * 0.25, soft_total * 0.5, soft_total * 0.25)
        module_joint_groups = MODULE_JOINT_GROUPS
        joints_per_module = 2
    elif joint_count == 16:
        if len(joints) == 8:
            expanded = []
            for q in joints:
                expanded.extend((0.5 * q, 0.5 * q))
            joints = expanded
        elif len(joints) != 16:
            raise ValueError("Expected 8 or 16 joints for 16-joint mode, got {}".format(len(joints)))
        soft_lengths = (soft_l1, soft_l2, soft_l3, soft_l4, soft_l5)
        module_joint_groups = PHYSICAL_MODULE_JOINT_GROUPS
        joints_per_module = 4
    else:
        raise ValueError("Unsupported joint_count: {}".format(joint_count))

    p = (0.0, 0.0)
    r = ((1.0, 0.0), (0.0, 1.0))
    chain_points = [p]
    soft_link_roots = [("soft_link1", p)]
    joint_points = []
    gimbal_points = []
    rotor_points = []

    for module_i, module in enumerate(module_params):
        module_offset = rotate_vec(r, (module["parent_to_soft_root_x"], 0.0))
        p = (p[0] + module_offset[0], p[1] + module_offset[1])
        chain_points.append(p)

        module_joint_names = module_joint_groups[module_i]
        joint_start = joints_per_module * module_i
        module_joints = joints[joint_start : joint_start + joints_per_module]

        for seg_i, seg_len in enumerate(soft_lengths):
            link_name = "soft_link{}".format(len(soft_lengths) * module_i + seg_i + 1)
            if module_i == 0 and seg_i == 0:
                soft_link_roots[0] = (link_name, p)
            else:
                soft_link_roots.append((link_name, p))

            link = rotate_vec(r, (seg_len, 0.0))
            p = (p[0] + link[0], p[1] + link[1])
            chain_points.append(p)

            if seg_i < len(module_joints):
                joint_points.append((module_joint_names[seg_i], p))
                r = mat_mul(r, rot(module_joints[seg_i]))

        servo_tail = rotate_vec(r, (module["servo_size_x"], 0.0))
        p = (p[0] + servo_tail[0], p[1] + servo_tail[1])
        chain_points.append(p)
        gimbal_points.append(p)

        rotor_offset = rotate_vec(r, (rotor_offset_x, 0.0))
        rotor_points.append((p[0] + rotor_offset[0], p[1] + rotor_offset[1]))

    return {
        "chain": chain_points,
        "soft_link_roots": soft_link_roots,
        "joint_points": joint_points,
        "gimbals": gimbal_points,
        "rotors": rotor_points,
    }


def compute_fixed_plot_limits(
    xlim=DEFAULT_XLIM,
    ylim=DEFAULT_YLIM,
):
    return xlim, ylim


def extract_servo_map(msg):
    return {int(servo.index): int(servo.angle) for servo in msg.servos}


def find_closest_servo_state(bag_path, servo_topic, seconds):
    with rosbag.Bag(bag_path, "r") as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        target_time = start_time + seconds
        closest = None
        for _, msg, t in bag.read_messages(topics=[servo_topic]):
            sample_time = t.to_sec()
            dt = abs(sample_time - target_time)
            if closest is None or dt < closest["dt"]:
                closest = {
                    "msg": msg,
                    "time": sample_time,
                    "dt": dt,
                    "start_time": start_time,
                    "end_time": end_time,
                    "target_time": target_time,
                }

    if closest is None:
        raise RuntimeError("No messages found on topic '{}'".format(servo_topic))

    return closest


def plot_shape(ax, shape, title):
    chain = shape["chain"]
    xs = [p[0] for p in chain]
    ys = [p[1] for p in chain]
    gx = [p[0] for p in shape["gimbals"]]
    gy = [p[1] for p in shape["gimbals"]]
    rx = [p[0] for p in shape["rotors"]]
    ry = [p[1] for p in shape["rotors"]]

    ax.clear()
    ax.plot(xs, ys, "-o", linewidth=2.0, markersize=3.5, color="#1f77b4", label="shape")
    ax.scatter(gx, gy, s=40, c="#2ca02c", label="gimbal")
    ax.scatter(rx, ry, s=70, marker="x", c="#d62728", label="rotor")

    for name, p in shape["soft_link_roots"]:
        ax.text(p[0], p[1], name, fontsize=8, color="#444444", ha="left", va="bottom")
    for name, p in shape["joint_points"]:
        ax.text(p[0], p[1], name, fontsize=8, color="#9467bd", ha="right", va="bottom")

    ax.scatter([0.0], [0.0], s=45, c="#000000", label="soft_link1 origin")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.axis("equal")
    xlim, ylim = compute_fixed_plot_limits()
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.grid(True, linestyle=":", linewidth=0.8)
    ax.legend(loc="best")


def print_summary(sample, joints, servo_map):
    print(
        "bag_time={:.6f} s, target_time={:.6f} s, delta={:+.6f} s".format(
            sample["time"],
            sample["target_time"],
            sample["time"] - sample["target_time"],
        )
    )
    print(
        "bag_start={:.6f} s, requested_offset={:.6f} s, bag_duration={:.6f} s".format(
            sample["start_time"],
            sample["target_time"] - sample["start_time"],
            sample["end_time"] - sample["start_time"],
        )
    )
    print("servo_angles:")
    print("  {}".format(", ".join("{}:{}".format(idx, servo_map[idx]) for idx in sorted(servo_map))))
    print("estimated_joints[rad]:")
    for name, q in zip([j for group in MODULE_JOINT_GROUPS for j in group], joints):
        print("  {:>12s}: {:+.6f}".format(name, q))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot an approximated planar Hydrus soft-airframe shape at a given rosbag time."
    )
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--seconds", type=float, required=True, help="Seconds from bag start")
    parser.add_argument("--servo-topic", default="servo/states", help="ServoStates topic name")
    parser.add_argument("--servo-center", type=int, default=2047, help="Servo neutral encoder tick")
    parser.add_argument(
        "--max-joint-abs-rad",
        type=float,
        default=1.5,
        help="Maximum absolute logical joint angle used in estimation",
    )
    parser.add_argument(
        "--weight-previous",
        type=float,
        default=0.5,
        help="Regularization weight passed to the estimator",
    )
    parser.add_argument(
        "--ik-max-iters",
        type=int,
        default=120,
        help="Maximum SLSQP iterations for joint estimation",
    )
    parser.add_argument(
        "--joint-count",
        type=int,
        choices=(8, 16),
        default=8,
        help="Planar visualization joint count, following fk.py/fk2 style",
    )
    parser.add_argument("--output", help="Optional output image path")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open an interactive window even if DISPLAY is available",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    sample = find_closest_servo_state(args.bag, args.servo_topic, args.seconds)
    servo_map = extract_servo_map(sample["msg"])
    joints, _, res = estimate_all_modules_from_servo_map(
        servo_map,
        servo_center=args.servo_center,
        max_joint_abs_rad=args.max_joint_abs_rad,
        weight_previous=args.weight_previous,
        ik_max_iters=args.ik_max_iters,
    )
    if not res.success:
        print("warning: optimization did not fully converge: {}".format(res.message))

    shape = compute_planar_shape(joints, joint_count=args.joint_count)
    print_summary(sample, joints, servo_map)

    fig, ax = plt.subplots(figsize=DEFAULT_FIGSIZE)
    title = "Hydrus soft shape at {:.3f}s from bag start ({} joints)".format(
        args.seconds,
        args.joint_count,
    )
    plot_shape(ax, shape, title)
    plt.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=150)
        print("saved plot to {}".format(args.output))

    if not args.no_show and os.environ.get("DISPLAY"):
        plt.show()
    else:
        if not args.output:
            print("plot window was not opened; pass --output to save the figure in headless mode")
        plt.close(fig)


if __name__ == "__main__":
    main()
