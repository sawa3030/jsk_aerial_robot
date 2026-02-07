#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest


class BoxHoldDemoNode:
    def __init__(self):
        self.joint_states_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
        self.joint_states_sub = rospy.Subscriber("joint_states", JointState, self.joint_states_cb, queue_size=1)
        self.update_hz = 50
        self.positions = [0.8, 0.8, 0.8]
        self.joint1_position = 0.0
        self.joint2_position = 0.0
        self.first_callback = True

        self.srv_name = "add_extra_module"
        rospy.loginfo(f"Waiting for service: {self.srv_name}")
        rospy.wait_for_service(self.srv_name)
        self.add_extra_module = rospy.ServiceProxy(self.srv_name, AddExtraModule)
    
    def joint_states_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == "joint1":
                self.joint1_position = pos
            elif name == "joint2":
                self.joint2_position = pos

    def call_add_extra_module(self) -> bool:
        req = AddExtraModuleRequest()
        req.action = 1
        req.module_name = "box"
        req.parent_link_name = "soft_link3"

        # transform
        req.transform.translation.x = 0.1175
        req.transform.translation.y = 0.0
        req.transform.translation.z = 0.0
        req.transform.rotation.x = 0.0
        req.transform.rotation.y = 0.0
        req.transform.rotation.z = 0.0
        req.transform.rotation.w = 1.0

        # inertia
        req.inertia.m = 0.25
        req.inertia.com.x = 0.0
        req.inertia.com.y = 0.0
        req.inertia.com.z = 0.0
        req.inertia.ixx = 0.0001
        req.inertia.ixy = 0.0
        req.inertia.ixz = 0.0
        req.inertia.iyy = 0.0001
        req.inertia.iyz = 0.0
        req.inertia.izz = 0.0001

        try:
            resp = self.add_extra_module(req)
            rospy.loginfo(f"add_extra_module status: {resp.status}")
            return bool(resp.status)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def run(self):
        rospy.sleep(0.5)

        r = rospy.Rate(self.update_hz)

        rospy.sleep(1.0)
        self.positions = [self.joint1_position, self.joint2_position, self.joint1_position]

        while not rospy.is_shutdown():
            joint_msg = JointState()
            joint_msg.name = ["joint1", "joint2", "joint3"]
            joint_msg.position = self.positions
            joint_msg.header.stamp = rospy.Time.now()
            self.joint_states_pub.publish(joint_msg)
            print("current position:", self.positions)
            self.positions[0] += 0.01
            self.positions[2] += 0.01

            if self.positions[0] > 1.6:
                rospy.sleep(1.0)
                # self.call_add_extra_module()
                break
            
            # if self.positions[0] < 0.95:
                # rospy.sleep(5.0)
                # user_input = input("Press Enter to continue...")
                # if user_input != "":
                #     rospy.loginfo("Demo interrupted by user.")
                #     return
            if self.first_callback:
                self.first_callback = False
                user_input = input("Press Enter to continue...")
             
            rospy.sleep(0.5)
            r.sleep()



def main():
    rospy.init_node("box_hold_demo_node")
    node = BoxHoldDemoNode()
    node.run()

if __name__ == "__main__":
    main()
