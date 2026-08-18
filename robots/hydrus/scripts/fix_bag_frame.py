#!/usr/bin/env python
import sys
import rosbag
from nav_msgs.msg import Path

inbag = sys.argv[1]
outbag = sys.argv[2]

with rosbag.Bag(outbag, 'w') as out:
    for topic, msg, t in rosbag.Bag(inbag, 'r'):
        if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'):
            if msg.header.frame_id == '/world':
                msg.header.frame_id = 'world'

        if isinstance(msg, Path):
            for pose in msg.poses:
                if pose.header.frame_id == '/world':
                    pose.header.frame_id = 'world'

        out.write(topic, msg, t)