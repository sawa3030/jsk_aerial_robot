#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2025, DRAGON Laboratory, The University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import numpy as np
import rospy

from aerial_robot_base.robot_interface import RobotInterface


def wait_for_enter(prompt):
    try:
        raw_input(prompt)
    except NameError:
        input(prompt)


class RoundTrajectoryDemo(object):
    def __init__(self):
        self.robot = RobotInterface()
        rospy.sleep(1.0)

        self.center = np.array(rospy.get_param("~center", [0.0, -0.25, 0.6]), dtype=float)
        self.radius = float(rospy.get_param("~radius", 0.75))
        self.angular_velocity = float(rospy.get_param("~angular_velocity", 0.25))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.yaw = float(rospy.get_param("~yaw", 0))
        self.pos_thresh = float(rospy.get_param("~pos_thresh", 0.05))
        self.vel_thresh = float(rospy.get_param("~vel_thresh", 0.05))
        self.start_phase = math.pi

        self.start_pos = self.center + np.array(
            [
                self.radius * math.cos(self.start_phase),
                self.radius * math.sin(self.start_phase),
                0.0,
            ]
        )

        rospy.loginfo(
            "Round trajectory demo: start=%s, center=%s, radius=%.3f, omega=%.3f",
            self.start_pos.tolist(),
            self.center.tolist(),
            self.radius,
            self.angular_velocity,
        )

    def go_pos_yaw_timeout_is_reached(self, pos, yaw, timeout):
        if not self.robot.navigate(
            pos=pos,
            rot=(0.0, 0.0, yaw),
            pos_thresh=self.pos_thresh,
            vel_thresh=self.vel_thresh,
            rot_thresh=0.1,
            timeout=-1,
        ):
            return False

        start_time = rospy.get_time()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.robot.poseThresholdCheck(
                target_pos=pos,
                target_rot=(0.0, 0.0, yaw),
                pos_thresh=self.pos_thresh,
                vel_thresh=self.vel_thresh,
                rot_thresh=0.1,
            ):
                return True

            if rospy.get_time() - start_time > timeout:
                rospy.logwarn(
                    "Timeout while moving to %s. Treat this as reached and continue.",
                    np.asarray(pos).tolist(),
                )
                return True

            rate.sleep()

    def run(self):
        wait_for_enter("Press Enter to move to the circular trajectory start point...")
        if not self.go_pos_yaw_timeout_is_reached(
            self.start_pos,
            self.yaw,
            timeout=15,
        ):
            rospy.logerr("Failed to reach the start position: %s", self.start_pos.tolist())
            return

        wait_for_enter("Press Enter to start circular tracking...")
        self.track_circle()

    def track_circle(self):
        if self.angular_velocity == 0.0:
            rospy.logerr("angular_velocity must be non-zero to track a circle.")
            return

        rate = rospy.Rate(self.control_rate)
        start_time = rospy.Time.now().to_sec()
        duration = 2.0 * math.pi / abs(self.angular_velocity)

        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed >= duration:
                break

            theta = self.start_phase + self.angular_velocity * elapsed

            pos = self.center + np.array(
                [
                    self.radius * math.cos(theta),
                    self.radius * math.sin(theta),
                    0.0,
                ]
            )
            vel = np.array(
                [
                    -self.radius * self.angular_velocity * math.sin(theta),
                    self.radius * self.angular_velocity * math.cos(theta),
                    0.0,
                ]
            )

            self.robot.navigate(
                pos=pos,
                rot=(0.0, 0.0, self.yaw),
                lin_vel=vel,
                pos_thresh=self.pos_thresh,
                vel_thresh=self.vel_thresh,
                timeout=-1,
            )
            rate.sleep()

        self.go_pos_yaw_timeout_is_reached(
            self.start_pos,
            self.yaw,
            timeout=10,
        )


if __name__ == "__main__":
    rospy.init_node("round_trajectory_demo")
    demo = RoundTrajectoryDemo()
    demo.run()
