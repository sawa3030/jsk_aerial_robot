#!/bin/bash

rosrun aerial_robot_base rosbag_control_data.sh ${1:-uuv_d} ${1:-uuv_d}/zed/odom ${1:-uuv_d}/realsense1/odom/throttle ${1:-uuv_d}/realsense1/odom ${1:-uuv_d}/realsense2/odom/throttle ${1:-uuv_d}/realsense2/odom ${@:2}
