#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

DEFAULT_MODULE_PARAMS = [
    {"parent_to_servo_x": 0.0, "servo_size_x": 0.096},
    {"parent_to_servo_x": 0.147, "servo_size_x": 0.156},
    {"parent_to_servo_x": 0.147, "servo_size_x": 0.096},
    {"parent_to_servo_x": 0.147, "servo_size_x": 0.096},
]
# from robots/hydrus/urdf/soft_link.urdf.xacro (soft_link_module defaults)
DEFAULT_SOFT_L1 = 0.05875
DEFAULT_SOFT_L2 = 0.1175
DEFAULT_SOFT_L3 = 0.1175
DEFAULT_SOFT_L4 = 0.1175
DEFAULT_SOFT_L5 = 0.05875


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


def compute_end_pose_error_sq(
    joints,
    module_params=None,
    soft_l1=DEFAULT_SOFT_L1,
    soft_l2=DEFAULT_SOFT_L2,
    soft_l3=DEFAULT_SOFT_L3,
    soft_l4=DEFAULT_SOFT_L4,
    soft_l5=DEFAULT_SOFT_L5,
):
    if module_params is None:
        module_params = DEFAULT_MODULE_PARAMS

    p = (0.0, 0.0)
    r = ((1.0, 0.0), (0.0, 1.0))
    theta_raw = 0.0

    soft_lengths = [soft_l1, soft_l2, soft_l3, soft_l4, soft_l5]

    for module_i, module in enumerate(module_params):
        module_offset = rotate_vec(r, (module["parent_to_servo_x"], 0.0))
        p = (p[0] + module_offset[0], p[1] + module_offset[1])

        module_joints = joints[4 * module_i : 4 * module_i + 4]
        for link_i, link_len in enumerate(soft_lengths):
            link = rotate_vec(r, (link_len, 0.0))
            p = (p[0] + link[0], p[1] + link[1])
            if link_i < len(module_joints):
                q = module_joints[link_i]
                theta_raw += q
                r = mat_mul(r, rot(q))

        tail = rotate_vec(r, (module["servo_size_x"], 0.0))
        p = (p[0] + tail[0], p[1] + tail[1])

    end_x, end_y = p
    pose_pos_err2 = end_x * end_x + end_y * end_y
    sum_360_err2 = (theta_raw - 2.0 * math.pi) ** 2
    return pose_pos_err2, sum_360_err2
