#!/usr/bin/env python3

import math

import rospy
from spinal.msg import Imu


class ImuToRollPitchPrinter(object):
    def __init__(self):
        rospy.init_node("convert_imu_to_rpy")

        self.imu_topic = rospy.get_param("~imu_topic", "/imu/neuron2")
        self.use_degree = rospy.get_param("~degree", True)
        self.alpha = rospy.get_param("~alpha", 0.98)

        self.roll = 0.0
        self.pitch = 0.0
        self.prev_stamp = None
        self.initialized = False

        self.sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=1)

        rospy.loginfo("subscribe imu topic: %s", self.imu_topic)
        rospy.loginfo("estimate roll/pitch with complementary filter (alpha=%.3f)", self.alpha)

    def imu_callback(self, msg):
        ax, ay, az = msg.acc
        gx, gy, _ = msg.gyro

        stamp = msg.stamp if msg.stamp.to_sec() > 0.0 else rospy.Time.now()

        acc_roll = math.atan2(ay, az)
        acc_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        if not self.initialized:
            self.roll = acc_roll
            self.pitch = acc_pitch
            self.prev_stamp = stamp
            self.initialized = True
            self.print_estimation(stamp)
            return

        dt = (stamp - self.prev_stamp).to_sec()
        self.prev_stamp = stamp
        if dt <= 0.0:
            return

        gyro_roll = self.roll + gx * dt
        gyro_pitch = self.pitch + gy * dt

        self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * acc_roll
        self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * acc_pitch

        self.print_estimation(stamp)

    def print_estimation(self, stamp):
        roll = self.roll
        pitch = self.pitch

        if self.use_degree:
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            unit = "deg"
        else:
            unit = "rad"

        print(
            "[{:.3f}] pitch: {:.3f} {}, roll: {:.3f} {}".format(
                stamp.to_sec(), pitch, unit, roll, unit
            )
        )


def main():
    try:
        ImuToRollPitchPrinter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
