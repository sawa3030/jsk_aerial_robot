#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import copy
import cv2

from aerial_robot_base.robot_interface import RobotInterface
from aerial_robot_base.state_machine import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aerial_robot_msgs.msg import FlightNav
from message_filters import ApproximateTimeSynchronizer, Subscriber
# import cv2
import numpy as np


class BlockPickDemo():
    def __init__(self):
        # self.ri = RobotInterface()
        rospy.sleep(1.0) # wait for joint updated
        self.bridge = CvBridge()

        self.depth_min = 0.00001
        self.depth_max = 30.0
        self.min_area = 10
        self.points_num = 100
        self.points_var = 100
        self.scale = 0.001
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

        # rospy.Subscriber("/rs_d435/color/image_rect_color", Image, self.cb_image)
        # rospy.Subscriber("/rs_d435/aligned_depth_to_color/image_raw", Image, self.cb_depth)
        # rospy.Subscriber("/camera/color/image_raw", Image, self.cb_image)
        # rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.cb_depth)
        image_sub = Subscriber("/camera/color/image_raw", Image)
        depth_sub = Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        # image_sub = Subscriber("/rs_d435/color/image_rect_color", Image)
        # depth_sub = Subscriber("/rs_d435/aligned_depth_to_color/image_raw", Image)

        tss = ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=10, slop=0.01)
        tss.registerCallback(self.gotimage)

        self.nav_pub = rospy.Publisher('/hydrus/uav/nav', FlightNav, queue_size=1)
        self.recognize_pub = rospy.Publisher('/hydrus/recognize', Image, queue_size=1)

        self.update_hz = 50

    def gotimage(self, image, depth_image):
        self.image = image
        self.depth_image = depth_image

    def getDepthOfRedObject(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # === Filter red color ===

            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)

            mask = cv2.bitwise_or(mask1, mask2)

            # === Get the largest and get the center ===
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                return
            max_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(max_contour)

            if M["m00"] < self.min_area:
                return
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            self.image_height, self.image_width = mask.shape
            self.cx = cx
            self.cy = cy
            print(f"最大輪郭の重心座標: ({cx}, {cy})")
            
            # === Get the points near the center ===
            rand_offsets = (np.random.rand(self.points_num, 2) - 0.5) * self.points_var
            candidate_points = np.array([self.cx, self.cy]) + rand_offsets
            candidate_points = candidate_points.astype(int)

            valid_mask = (
                (0 <= candidate_points[:, 0]) & (candidate_points[:, 0] < self.image_width) &
                (0 <= candidate_points[:, 1]) & (candidate_points[:, 1] < self.image_height)
            )
            candidate_points = candidate_points[valid_mask]

            ys = candidate_points[:, 1]
            xs = candidate_points[:, 0]
            mask_values = mask[ys, xs]

            valid_mask_points = mask_values > 0
            points_x = xs[valid_mask_points]
            points_y = ys[valid_mask_points]

            if len(points_x) == 0:
                return

        except Exception as e:
            print(e)
            return

        try:
            # === Get the average of the depth near the center ===
            depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding='passthrough')
            height, width = depth_image.shape
            points_x = (points_x * width) // self.image_width
            points_y = (points_y * height) // self.image_height

            depths = []
            for i in range(len(points_x)):
                if points_x[i] < width and points_y[i] < height:
                    depth_of_each_point = depth_image[points_y[i], points_x[i]] * self.scale
                    if depth_of_each_point > 0.0:
                        depths.append(depth_of_each_point)
            self.update_time_depth = self.depth_image.header.stamp
            self.depth = np.mean(depths)
        except Exception as e:
            print(e)
            return
            

    def run(self):
        r = rospy.Rate(self.update_hz)

        while not rospy.is_shutdown():
            # user code begin
            # self.ri.setJointAngle(["joint1", "joint3"], [1.0, 1.0])
            if hasattr(self, "image"):
                self.getDepthOfRedObject()
                if not hasattr(self, "depth"):
                    continue
                print(f"depth: {self.depth}")

                # if (rospy.Time.now() - self.update_time_depth).to_sec() < 1.0:
                #     if self.depth < self.depth_min or self.depth_max < self.depth:
                #         continue

                #     nav_msg = FlightNav()
                #     nav_msg.control_frame = FlightNav.LOCAL_FRAME
                #     nav_msg.target = FlightNav.COG

                #     if self.cx < self.image_width / 3:
                #         nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                #         nav_msg.target_omega_z  = 0.1
                #         print("turning right")
                #     elif self.cx > self.image_width * 2 / 3:
                #         nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                #         nav_msg.target_omega_z = -0.1
                #         print("turning left")
                #     else:
                #         nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                #         nav_msg.target_vel_x = -0.1
                #         nav_msg.target_vel_y = 0.1
                #         print("moving forward")
                #     self.nav_pub.publish(nav_msg)

            # user code end

            r.sleep()


if __name__ == "__main__":
    rospy.init_node("hydrus_demo")
    node = BlockPickDemo()
    node.run()