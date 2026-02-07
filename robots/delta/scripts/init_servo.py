#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
from spinal.msg import ServoControlCmd, ServoStates

class InitSoftJointToServoNode:
    def __init__(self):
        rospy.init_node("init_soft_joint_to_servo")
        rospy.sleep(1.0) 

        self.servo_center = 2047
        self.count = 1000

        self.pub = rospy.Publisher("servo/target_states", ServoControlCmd, queue_size=1)
        self.sub = rospy.Subscriber("servo/states", ServoStates, self.cb, queue_size=1)

        self._headers = ["angle", "temperature", "load"]
        self._servo_num = 15
        self._table_data = [
            {"angle": None, "temperature": None, "load": None}
            for _ in range(self._servo_num)
        ]
        self._servo_num_of_interest = [7,8,9,10,11,12,13,14]  # Example servo indices of interest
        self._sigma = [-1, +1, +1 ,-1, -1, 1, 1, -1]
        self._threshold_load = [20, 20, 300, 20, 20, 20, 20, 300]

    
    def cb(self, msg):
        self.count += 1
        if self.count < 10:
            return  # wait for some messages to stabilize
        self.count = 0

        cnt = 0
        for s in msg.servos:
            # process to avoid to read non exist servo
            cnt +=1
            if cnt > self._servo_num:
                return
            self._table_data[s.index][self._headers.index("angle")] = s.angle
            self._table_data[s.index][self._headers.index("temperature")] = s.temp
            self._table_data[s.index][self._headers.index("load")] = s.load

        
        out = ServoControlCmd()
        out.index = []
        out.angles = []
        for i in self._servo_num_of_interest:
            if self._table_data[i] is None:
                rospy.logwarn_throttle(1.0, f"Servo {i} data is missing.")
                return
            if abs(self._table_data[i][self._headers.index("load")]) < self._threshold_load[self._servo_num_of_interest.index(i)]:
                out.index.append(i)
                out.angles.append(self._table_data[i][self._headers.index("angle")] - 100 * self._sigma[self._servo_num_of_interest.index(i)])
        
        self.pub.publish(out)
        rospy.loginfo_throttle(0.5, f"Published servo angles for initialization: {out.angles}")

def main():
    try:
        InitSoftJointToServoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
