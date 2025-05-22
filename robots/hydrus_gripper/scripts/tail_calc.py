import numpy as np
import math

s = 110
d = 5
r_joint_1 = 75 / 2 / math.sqrt(2)
r_joint_2 = 75 / 2

# p = np.array(
#     [
#         [0.0],
#         [0.0],
#         [115*4],
#     ]
# )


def solve_ik(alpha_1, alpha_2, alpha_3, alpha_4, p_des):
    for i in range(100):
        print("num of iteration: ", i)
        R_0_to_1 = np.array(
            [
                [math.cos(alpha_1), 0, math.sin(alpha_1)],
                [0, 1, 0],
                [-math.sin(alpha_1), 0, math.cos(alpha_1)],
            ]
        )
        R_1_to_2 = np.array(
            [
                [1, 0, 0],
                [0, math.cos(alpha_2), math.sin(alpha_2)],
                [0, -math.sin(alpha_2), math.cos(alpha_2)],
            ]
        )
        R_2_to_3 = np.array(
            [
                [math.cos(alpha_3), 0, -math.sin(alpha_3)],
                [0, 1, 0],
                [math.sin(alpha_3), 0, math.cos(alpha_3)],
            ]
        )
        R_3_to_4 = np.array(
            [
                [1, 0, 0],
                [0, math.cos(alpha_4), math.sin(alpha_4)],
                [0, -math.sin(alpha_4), math.cos(alpha_4)],
            ]
        )

        R_0_to_1_prime = np.array(
            [
                [-math.sin(alpha_1), 0, math.cos(alpha_1)],
                [0, 0, 0],
                [-math.cos(alpha_1), 0, -math.sin(alpha_1)],
            ]
        )
        R_1_to_2_prime = np.array(
            [
                [0, 0, 0],
                [0, -math.sin(alpha_2), math.cos(alpha_2)],
                [0, -math.cos(alpha_2), -math.sin(alpha_2)],
            ]
        )
        R_2_to_3_prime = np.array(
            [
                [-math.sin(alpha_3), 0, -math.cos(alpha_3)],
                [0, 0, 0],
                [math.cos(alpha_3), 0, -math.sin(alpha_3)],
            ]
        )
        R_3_to_4_prime = np.array(
            [
                [0, 0, 0],
                [0, -math.sin(alpha_4), math.cos(alpha_4)],
                [0, -math.cos(alpha_4), -math.sin(alpha_4)],
            ]
        )

        if alpha_1 == 0:
            p_0_to_1 = np.array(
                [
                    [0],
                    [0],
                    [s + d],
                ]
            )
            p_0_to_1_prime = np.array(
                [
                    [0],
                    [0],
                    [0],
                ]
            )
        else:
            r_1 = s / alpha_1
            p_0_to_1 = np.array(
                [
                    [r_1 * (1 - math.cos(alpha_1)) + d * math.sin(alpha_1)],
                    [0],
                    [r_1 * math.sin(alpha_1) + d * math.cos(alpha_1)],
                ]
            )
            p_0_to_1_prime = np.array(
                [
                    [
                        -s / alpha_1 / alpha_1 * (1 - math.cos(alpha_1))
                        + r_1 * math.sin(alpha_1)
                        + d * math.cos(alpha_1)
                    ],
                    [0],
                    [
                        -s / alpha_1 / alpha_1 * math.sin(alpha_1)
                        + r_1 * math.cos(alpha_1)
                        - d * math.sin(alpha_1)
                    ],
                ]
            )
        if alpha_2 == 0:
            p_1_to_2 = np.array(
                [
                    [0],
                    [0],
                    [s + d],
                ]
            )
            p_1_to_2_prime = np.array(
                [
                    [0],
                    [0],
                    [0],
                ]
            )
        else:
            r_2 = s / alpha_2
            p_1_to_2 = np.array(
                [
                    [0],
                    [r_2 * (1 - math.cos(alpha_2)) + d * math.sin(alpha_2)],
                    [r_2 * math.sin(alpha_2) + d * math.cos(alpha_2)],
                ]
            )
            p_1_to_2_prime = np.array(
                [
                    [0],
                    [
                        -s / alpha_2 / alpha_2 * (1 - math.cos(alpha_2))
                        + r_2 * math.sin(alpha_2)
                        + d * math.cos(alpha_2)
                    ],
                    [
                        -s / alpha_2 / alpha_2 * math.sin(alpha_2)
                        + r_2 * math.cos(alpha_2)
                        - d * math.sin(alpha_2)
                    ],
                ]
            )
        if alpha_3 == 0:
            p_2_to_3 = np.array(
                [
                    [0],
                    [0],
                    [s + d],
                ]
            )
            p_2_to_3_prime = np.array(
                [
                    [0],
                    [0],
                    [0],
                ]
            )
        else:
            r_3 = s / alpha_3
            p_2_to_3 = np.array(
                [
                    [r_3 * (1 - math.cos(alpha_3)) + d * math.sin(alpha_3)],
                    [0],
                    [r_3 * math.sin(alpha_3) + d * math.cos(alpha_3)],
                ]
            )
            p_2_to_3_prime = np.array(
                [
                    [
                        -s / alpha_3 / alpha_3 * (1 - math.cos(alpha_3))
                        + r_3 * math.sin(alpha_3)
                        + d * math.cos(alpha_3)
                    ],
                    [0],
                    [
                        -s / alpha_3 / alpha_3 * math.sin(alpha_3)
                        + r_3 * math.cos(alpha_3)
                        - d * math.sin(alpha_3)
                    ],
                ]
            )
        if alpha_4 == 0:
            p_3_to_4 = np.array(
                [
                    [0],
                    [0],
                    [s + d],
                ]
            )
            p_3_to_4_prime = np.array(
                [
                    [0],
                    [0],
                    [0],
                ]
            )
        else:
            r_4 = s / alpha_4
            p_3_to_4 = np.array(
                [
                    [0],
                    [r_4 * (1 - math.cos(alpha_4)) + d * math.sin(alpha_4)],
                    [r_4 * math.sin(alpha_4) + d * math.cos(alpha_4)],
                ]
            )
            p_3_to_4_prime = np.array(
                [
                    [0],
                    [
                        -s / alpha_4 / alpha_4 * (1 - math.cos(alpha_4))
                        + r_4 * math.sin(alpha_4)
                        + d * math.cos(alpha_4)
                    ],
                    [
                        -s / alpha_4 / alpha_4 * math.sin(alpha_4)
                        + r_4 * math.cos(alpha_4)
                        - d * math.sin(alpha_4)
                    ],
                ]
            )

        p = (
            R_0_to_1 @ p_0_to_1
            + R_0_to_1 @ R_1_to_2 @ p_1_to_2
            + R_0_to_1 @ R_1_to_2 @ R_2_to_3 @ p_2_to_3
            + R_0_to_1 @ R_1_to_2 @ R_2_to_3 @ R_3_to_4 @ p_3_to_4
        )
        if np.linalg.norm(p_des - p) < 13:
            print("alpha_1: ", math.degrees(alpha_1))
            print("alpha_2: ", math.degrees(alpha_2))
            print("alpha_3: ", math.degrees(alpha_3))
            print("alpha_4: ", math.degrees(alpha_4))
            return (alpha_1, alpha_2, alpha_3, alpha_4)

        dp_dalpha_1 = (
            p_0_to_1_prime
            + R_0_to_1_prime @ p_1_to_2
            + R_0_to_1_prime @ R_1_to_2 @ p_2_to_3
            + R_0_to_1 @ R_1_to_2 @ p_2_to_3_prime
            + R_0_to_1_prime @ R_1_to_2 @ R_2_to_3 @ p_3_to_4
            + R_0_to_1 @ R_1_to_2 @ R_2_to_3_prime @ p_3_to_4
        )
        dp_dalpha_2 = (
            R_0_to_1 @ p_1_to_2_prime
            + R_0_to_1 @ R_1_to_2_prime @ p_2_to_3
            + R_0_to_1 @ R_1_to_2_prime @ R_2_to_3 @ p_3_to_4
        )
        dp_dalpha_4 = R_0_to_1 @ R_1_to_2 @ R_2_to_3 @ p_3_to_4_prime

        dp_dalpha = np.concatenate((dp_dalpha_1, dp_dalpha_2, dp_dalpha_4), axis=1)
        dq = np.dot(np.linalg.pinv(dp_dalpha), (p_des - p))

        alpha_1 += dq[0, 0]
        alpha_2 += dq[1, 0]
        alpha_3 += dq[0, 0]
        alpha_4 += dq[2, 0]

        if i == 99:
            raise Exception("IK was not solved")


def get_wire_diff(alpha_1, alpha_2, alpha_3, alpha_4):
    def get_plus_pos_wire_length(alpha, r_joint):  # xまたはyが正のワイヤーの長さ
        if alpha == 0:
            return s + d
        r = s / alpha
        if alpha > 0:
            return 4 * (r - r_joint) * math.sin(alpha / 4) + d
        else:
            return 4 * (r + r_joint) * math.sin(alpha / 4) + d

    def get_minus_pos_wire_length(alpha, r_joint):  # xまたはyが負のワイヤーの長さ
        if alpha == 0:
            return s + d
        r = s / alpha
        if alpha > 0:
            return 4 * (r + r_joint) * math.sin(alpha / 4) + d
        else:
            return 4 * (r - r_joint) * math.sin(alpha / 4) + d

    x_plus_y_plus_wire = (
        get_plus_pos_wire_length(alpha_1, r_joint_1)
        + get_plus_pos_wire_length(alpha_2, r_joint_1)
        + get_plus_pos_wire_length(alpha_3, r_joint_1)
        + get_plus_pos_wire_length(alpha_4, r_joint_1)
    )
    x_plus_y_minus_wire = (
        get_plus_pos_wire_length(alpha_1, r_joint_1)
        + get_minus_pos_wire_length(alpha_2, r_joint_1)
        + get_plus_pos_wire_length(alpha_3, r_joint_1)
        + get_minus_pos_wire_length(alpha_4, r_joint_1)
    )
    x_minus_y_plus_wire = (
        get_minus_pos_wire_length(alpha_1, r_joint_1)
        + get_plus_pos_wire_length(alpha_2, r_joint_1)
        + get_minus_pos_wire_length(alpha_3, r_joint_1)
        + get_plus_pos_wire_length(alpha_4, r_joint_1)
    )
    x_minus_y_minus_wire = (
        get_minus_pos_wire_length(alpha_1, r_joint_1)
        + get_minus_pos_wire_length(alpha_2, r_joint_1)
        + get_minus_pos_wire_length(alpha_3, r_joint_1)
        + get_minus_pos_wire_length(alpha_4, r_joint_1)
    )
    x_zero_y_plus_wire = get_plus_pos_wire_length(
        alpha_1, 0
    ) + get_plus_pos_wire_length(alpha_2, r_joint_2)

    return (
        x_plus_y_plus_wire - (s + d) * 4,
        x_plus_y_minus_wire - (s + d) * 4,
        x_minus_y_plus_wire - (s + d) * 4,
        x_minus_y_minus_wire - (s + d) * 4,
        x_zero_y_plus_wire - (s + d) * 2,
    )


if __name__ == "__main__":
    alpha_1 = 0.0
    alpha_2 = 0.0
    alpha_3 = alpha_1
    alpha_4 = 0.0
    p_des = np.array(
        [
            [-200.0],
            [-100.0],
            [115 * 4 - 100],
        ]
    )

    dest_alpha_1, dest_alpha_2, dest_alpha_3, dest_alpha_4 = solve_ik(
        alpha_1, alpha_2, alpha_3, alpha_4, p_des
    )

    x_plus_y_plus_wire, x_plus_y_minus_wire, x_minus_y_plus_wire, x_minus_y_minus_wire, x_zero_y_plus_wire = get_wire_diff(
        dest_alpha_1, dest_alpha_2, dest_alpha_3, dest_alpha_4
    )  # x_plus_y_plus_wire, x_plus_y_minus_wire, x_minus_y_plus_wire, x_minus_y_minus_wire, x_zero_y_plus_wire
    print("x_plus_y_plus_wire: ", x_plus_y_plus_wire)
    print("x_plus_y_minus_wire: ", x_plus_y_minus_wire)
    print("x_minus_y_plus_wire: ", x_minus_y_plus_wire)
    print("x_minus_y_minus_wire: ", x_minus_y_minus_wire)
    print("x_zero_y_plus_wire: ", x_zero_y_plus_wire)
