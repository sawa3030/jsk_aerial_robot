#!/usr/bin/env python3

import math

import rospy
from sensor_msgs.msg import JointState


class PubSoftLinkJointTargetInterpolator(object):
    TARGET_JOINT_NAMES = [
        "soft_joint2",
        "soft_joint3",
        "soft_joint4",
        "soft_joint5",
        "soft_joint7",
        "soft_joint8",
        "soft_joint9",
        "soft_joint10",
        "soft_joint12",
        "soft_joint13",
        "soft_joint14",
        "soft_joint15",
        "soft_joint17",
        "soft_joint18",
        "soft_joint19",
        "soft_joint20",
    ]

    FIXED_TARGET_INITIALIZE = [
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0, 
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
        22.5 * math.pi / 180.0,
    ]

    # 四角・検証した
    FIXED_TARGET_SQUARE = [
        0.774,
        0.538,
        0.193,
        -0.081,
        -0.015,
        0.27,
        0.639,
        0.902,
        0.936,
        0.672,
        0.304,
        0.022,
        -0.147,
        0.12,
        0.456,
        0.688,
    ]

    # ハート型（rviz上で操作）
    # FIXED_TARGET = [
    #     0.5541769440932391, 
    #     1.1824954748111987, 
    #     0.4806636759992382, 
    #     0.515221195188726,
    #     0.0,
    #     0.14765485471871997,
    #     0.775973385436679,
    #     0.5912477374055989,
    #     0.5912477374055989, 
    #     0.6283185307179586, 
    #     0.11058406140636023, 
    #     -0.03707079331235974, 
    #     1.5890175641857178, 
    #     1.0719114134048375, 
    #     -0.5541769440932396,
    #     -0.7389025921243193,
    # ]

    # ハート型
    FIXED_TARGET_HEART = [
        0.43836899001000035, 
        0.4066350478067108, 
        0.3146641730208862, 
        0.34870962806378225, 
        0.356217568137726, 
        0.42446853446942934, 
        0.3428317651790263, 
        0.38229654185121625, 
        0.8388226235598808, 
        0.6962495372934097, 
        0.5491224327636147, 
        -0.26671517827601987, 
        -0.3110220607030547, 
        -0.02628257470745473, 
        0.8271568245691509, 
        1.0497102466424257,
    ]

    # バナナ型・検証した
    FIXED_TARGET_BANANA = [
        0.35430739133242517, 
        0.49542450708173463, 
        0.872327010803173, 
        1.090608944435494, 
        -0.32632777109260602, 
        -0.27611482230761377, 
        -0.112416749196029583, 
        -0.120912614675746767,
        0.9714066551700795, 
        0.865258353560416, 
        0.4934516820102892, 
        0.39800447877889933, 
        0.2799090880334772, 
        0.29974116448995536, 
        0.2909256570517672, 
        0.2775058170995593, 
    ]

    # ひょうたん型・検証済
    FIXED_TARGET_HYOTAN = [
        0.6319032719917186, 
        0.6747083935062192, 
        -0.03217889642088992, 
        -0.1709806088575641,
        -0.08678767447058687,
        0.2506095907464262,
        1.1464722722649892,
        1.112192152535221,
        0.7024190851016197,
        0.7305181262930987,
        -0.1657310365042583,
        -0.11463426490730001,
        -0.23227979333272689,
        -0.33692812239370007,
        1.190933913969154,
        1.1488552435402303
    ]

    def __init__(self):
        rospy.init_node("pub_soft_link_joint_target_interpolator")

        self.publish_hz = rospy.get_param("~publish_hz", 10.0)
        self.transition_time = max(1.0e-3, rospy.get_param("~transition_time", 50.0))
        self.output_topic = rospy.get_param("~output_topic", "soft_joint_reference_interp")
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "joint_states")
        self.joint_names = list(self.TARGET_JOINT_NAMES)
        self.current = [math.radians(22.5)] * len(self.joint_names)
        self.target = list(self.current)
        self.start = list(self.current)
        self.interp_start_time = rospy.Time.now()
        self.initialized_from_joint_states = False

        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=1)
        self.joint_states_sub = rospy.Subscriber(
            self.joint_states_topic, JointState, self.joint_states_cb, queue_size=1
        )

        mode = rospy.get_param("~mode", "preset")
        if mode != "preset":
            rospy.logwarn("unsupported mode '%s', fallback to 'preset'", mode)
        target_name=input("Enter target shape (s: square, b:banana, hy: hyotan, i: initialized): ").strip().lower()
        if target_name == "s":
            self.FIXED_TARGET = self.FIXED_TARGET_SQUARE
        elif target_name == "he":
            self.FIXED_TARGET = self.FIXED_TARGET_HEART
        elif target_name == "b": 
            self.FIXED_TARGET = self.FIXED_TARGET_BANANA
        elif target_name == "hy":
            self.FIXED_TARGET = self.FIXED_TARGET_HYOTAN
        elif target_name == "i":
            self.FIXED_TARGET = self.FIXED_TARGET_INITIALIZE
        else:
            rospy.logwarn("unsupported target shape '%s', fallback to 'square'", target_name)
            self.FIXED_TARGET = self.FIXED_TARGET_INITIALIZE
        self.target = list(self.FIXED_TARGET)
        self.start = list(self.current)
        self.interp_start_time = rospy.Time.now()

        rospy.loginfo("interpolator mode: preset")
        rospy.loginfo("publish interpolated target to: %s", self.output_topic)
        rospy.loginfo("subscribe current joints from: %s", self.joint_states_topic)

    def joint_states_cb(self, msg):
        if self.initialized_from_joint_states:
            return
        if not msg.name or not msg.position:
            return
        name_to_pos = {}
        max_len = min(len(msg.name), len(msg.position))
        for i in range(max_len):
            name_to_pos[msg.name[i]] = float(msg.position[i])

        if not all(name in name_to_pos for name in self.joint_names):
            return

        self.current = [name_to_pos[name] for name in self.joint_names]
        self.start = list(self.current)
        self.interp_start_time = rospy.Time.now()
        self.initialized_from_joint_states = True
        rospy.loginfo("initialized current joints from %s", self.joint_states_topic)

    def _blend_step(self):
        elapsed = (rospy.Time.now() - self.interp_start_time).to_sec()
        alpha = elapsed / self.transition_time
        alpha = max(0.0, min(1.0, alpha))
        for i in range(len(self.current)):
            self.current[i] = self.start[i] + alpha * (self.target[i] - self.start[i])

    def run(self):
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown():
            if not self.initialized_from_joint_states:
                rate.sleep()
                continue

            self._blend_step()

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = list(self.joint_names)
            msg.position = list(self.current)
            self.pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    node = PubSoftLinkJointTargetInterpolator()
    node.run()
