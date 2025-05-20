import numpy as np
import math

s = 0.5
d = 0.1
alpha_1 = 0.0
alpha_2 = 0.0
alpha_3 = alpha_1
alpha_4 = 0.0
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

