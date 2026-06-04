#!/bin/bash

if [ $# -ne 1 ];then
    echo "Usage: rosrun uuv_d rosbag_record.sh file_name_prefix";
    exit 1
fi

rosbag record -a -x ".*/cloud_registered.*|.*/compressedDepth.*|.*/theora.*|.*/image" -o $1