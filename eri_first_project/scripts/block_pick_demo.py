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
from message_filters import TimeSynchronizer, Subscriber
# import cv2
import numpy as np


class BlockPickDemo():
    def __init__(self):
        self.ri = RobotInterface()
        rospy.sleep(1.0) # wait for joint updated
        self.bridge = CvBridge()

        # rospy.Subscriber("/rs_d435/color/image_rect_color", Image, self.cb_image)
        # rospy.Subscriber("/rs_d435/aligned_depth_to_color/image_raw", Image, self.cb_depth)
        # rospy.Subscriber("/camera/color/image_raw", Image, self.cb_image)
        # rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.cb_depth)
        image_sub = Subscriber("/rs_d435/color/image_rect_color", Image)
        depth_sub = Subscriber("/rs_d435/aligned_depth_to_color/image_raw", Image)

        tss = TimeSynchronizer([image_sub, depth_sub], queue_size=10)
        tss.registerCallback(self.gotimage)

        self.nav_pub = rospy.Publisher('/hydrus/uav/nav', FlightNav, queue_size=1)

        self.update_hz = 50
    
    def gotimage(self, image, depth):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

            mask = cv2.bitwise_or(mask1, mask2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                return
            max_contour = max(contours, key=cv2.contourArea)

            M = cv2.moments(max_contour)

            if M["m00"] < 10: # this might needs to be tuned
                return
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            image_height, image_width = mask.shape
            self.cx = cx
            self.cy = cy
            print(f"最大輪郭の重心座標: ({cx}, {cy})")
            
            rand_offsets = (np.random.rand(100, 2) - 0.5) * 100  # this might needs to be tuned
            candidate_points = np.array([self.cx, self.cy]) + rand_offsets
            candidate_points = candidate_points.astype(int)

            valid_mask = (
                (0 <= candidate_points[:, 0]) & (candidate_points[:, 0] < image_width) &
                (0 <= candidate_points[:, 1]) & (candidate_points[:, 1] < image_height)
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

        except CvBridgeError as e:
            print(e)
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            scale = 0.001 # this might not be true
            height, width = depth_image.shape
            points_x = (points_x * width) // image_width
            points_y = (points_y * height) // image_height

            depths = []
            for i in range(len(points_x)):
                if points_x[i] < width and points_y[i] < height:
                    depths.append(depth_image[points_y[i], points_x[i]] * scale)
            self.depth = np.mean(depths)
            self.update_time_depth = depth.header.stamp
            print(f"depth: {self.depth}")
        except Exception as e:
            print(e)
            return
            

    def run(self):
        r = rospy.Rate(self.update_hz)

        while not rospy.is_shutdown():
            # user code begin
            # self.ri.setJointAngle(["joint1", "joint3"], [1.0, 1.0])
            if hasattr(self, "update_time_depth"):
                if (rospy.Time.now() - self.update_time_depth).to_sec() < 1.0:
                    if self.depth > 0.00001: # this might needs to be tuned
                        nav_msg = FlightNav()
                        nav_msg.control_frame = FlightNav.LOCAL_FRAME
                        nav_msg.target = FlightNav.COG
                        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                        nav_msg.target_vel_x = -0.1
                        nav_msg.target_vel_y = 0.1
                        self.nav_pub.publish(nav_msg)

            # user code end

            r.sleep()


if __name__ == "__main__":
    rospy.init_node("hydrus_demo")
    node = BlockPickDemo()
    node.run()