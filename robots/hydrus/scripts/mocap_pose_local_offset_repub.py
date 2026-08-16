#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft


class MocapPoseLocalOffsetRepublisher(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/hydrus/mocap/pose")
        self.output_topic = rospy.get_param("~output_topic", "/hydrus/mocap/pose_offset")
        self.offset_x = rospy.get_param("~offset_x", 0.15)
        self.offset_y = rospy.get_param("~offset_y", 0.0)
        self.offset_z = rospy.get_param("~offset_z", 0.0)

        self.pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PoseStamped, self.cb, queue_size=1)

        rospy.loginfo(
            "Republishing %s to %s with local offset [%.3f, %.3f, %.3f] m",
            self.input_topic,
            self.output_topic,
            self.offset_x,
            self.offset_y,
            self.offset_z,
        )

    def cb(self, msg):
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        rot = tft.quaternion_matrix(q)

        offset_world_x = (
            rot[0][0] * self.offset_x
            + rot[0][1] * self.offset_y
            + rot[0][2] * self.offset_z
        )
        offset_world_y = (
            rot[1][0] * self.offset_x
            + rot[1][1] * self.offset_y
            + rot[1][2] * self.offset_z
        )
        offset_world_z = (
            rot[2][0] * self.offset_x
            + rot[2][1] * self.offset_y
            + rot[2][2] * self.offset_z
        )

        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose
        out.pose.position.x += offset_world_x
        out.pose.position.y += offset_world_y
        out.pose.position.z += offset_world_z
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("mocap_pose_local_offset_republisher")
    MocapPoseLocalOffsetRepublisher()
    rospy.spin()
