#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
from scipy.optimize import minimize

class DeltaSoftLinkSolver:
    def __init__(self):
        rospy.init_node("delta_soft_ik_solver")
        
        self.joint_states_sub = rospy.Subscriber("joint_states", JointState, self.joint_states_cb, queue_size=1)
        self.joints_ctrl_sub = rospy.Subscriber("joints_ctrl", JointState, self.joints_ctrl_cb, queue_size=1)
        
        if self.is_simulation():
            self.joint_states_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
        else:
            self.joint_states_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        
        self.target_soft_joint_pub = rospy.Publisher("target_soft_joints_ctrl", JointState, queue_size=1)

        # --- リンク長パラメータ (単位: m) ---
        # 剛体リンク長 (Link1~4)
        self.L_RIGID = 0.5275 
        
        # --- 柔軟リンク長 (URDF解析結果) ---
        
        # 1. 始端: link4 -> soft_joint2
        self.L_S_START = 0.1175 
        
        # 2. 柔軟リンク間: soft_joint(N) -> soft_joint(N+1)
        # (s2->s3, s5->s6, s8->s9 はすべて共通)
        self.L_S_MID = 0.235  
        
        # 3. セクション接続部: soft_joint(End) -> soft_joint(NextStart)
        # 例: s3 -> s5 の距離
        # s3 -> gimbal(0.128) + gimbal -> s4_fixed(0.147) + s4 -> s5(0.1175)
        self.L_CONN = 0.128 + 0.147 + 0.1175 # = 0.3925
        
        # 4. 終端: soft_joint9 -> Tip
        # 閉ループ形成のため始端と同等と仮定
        self.L_END = 0.1175 

        # 最適化用初期値 (6自由度: s2, s3, s5, s6, s8, s9)
        self.last_sol = [0.0] * 6

        rospy.loginfo("Delta Soft IK Solver Initialized.")
        rospy.loginfo(f"Lengths -> Rigid:{self.L_RIGID}, Start:{self.L_S_START}, Mid:{self.L_S_MID}, Conn:{self.L_CONN}, End:{self.L_END}")

    def is_simulation(self) -> bool:
        return rospy.get_param('/use_sim_time', False)

    def joints_ctrl_cb(self, msg: JointState):
        # 剛体関節(joint1, joint2, joint3)の取得
        rigid_angles = [None, None, None] 
        for name, pos in zip(msg.name, msg.position):
            if name == 'joint1':
                rigid_angles[0] = pos
            elif name == 'joint2':
                rigid_angles[1] = pos
            elif name == 'joint3':
                rigid_angles[2] = pos
        if None in rigid_angles:
            rospy.logwarn("Not all rigid joints found in joint states")
            return

        # IKを解く
        soft_angles = self.solve_closed_loop_ik(rigid_angles)

        # Publish
        out_msg = JointState()
        out_msg.header.stamp = rospy.Time.now()
        # 出力対象: URDF内の全revolute soft joints
        out_msg.name = ["soft_joint2", "soft_joint3", 
                        "soft_joint5", "soft_joint6", 
                        "soft_joint8", "soft_joint9"]
        out_msg.position = soft_angles
        print("Published Soft Joint Angles:", soft_angles)
        self.target_soft_joint_pub.publish(out_msg)

    def joint_states_cb(self, msg: JointState):
        # 剛体関節(joint1, joint2, joint3)の取得
        rigid_angles = [None, None, None] 
        for name, pos in zip(msg.name, msg.position):
            if name == 'joint1':
                rigid_angles[0] = pos
            elif name == 'joint2':
                rigid_angles[1] = pos
            elif name == 'joint3':
                rigid_angles[2] = pos
        if None in rigid_angles:
            rospy.logwarn("Not all rigid joints found in joint states")
            return

        # IKを解く
        soft_angles = self.solve_closed_loop_ik(rigid_angles)

        # Publish
        out_msg = JointState()
        out_msg.header.stamp = rospy.Time.now()
        # 出力対象: URDF内の全revolute soft joints
        out_msg.name = ["soft_joint2", "soft_joint3", 
                        "soft_joint5", "soft_joint6", 
                        "soft_joint8", "soft_joint9"]
        out_msg.position = soft_angles
        print("Published Soft Joint Angles:", soft_angles)
        self.joint_states_pub.publish(out_msg)

    def solve_closed_loop_ik(self, rigid_joints):
        """
        剛体関節(j1, j2, j3)を入力とし、ループが閉じる柔軟関節(s2, s3, s5, s6, s8, s9)を求める。
        """
        j1, j2, j3 = rigid_joints

        def objective_func(soft_joints):
            # 変数展開: 6つの関節
            s2, s3, s5, s6, s8, s9 = soft_joints
            
            x, y, theta = 0.0, 0.0, 0.0
            
            # --- 1. Rigid Chain (Root -> Link4) ---
            # Link1 (Root->J1)
            x += self.L_RIGID * math.cos(theta)
            y += self.L_RIGID * math.sin(theta)
            theta += j1
            
            # Link2 (J1->J2)
            x += self.L_RIGID * math.cos(theta)
            y += self.L_RIGID * math.sin(theta)
            theta += j2
            
            # Link3 (J2->J3)
            x += self.L_RIGID * math.cos(theta)
            y += self.L_RIGID * math.sin(theta)
            theta += j3
            
            # Link4 (J3->SoftStart)
            x += self.L_RIGID * math.cos(theta)
            y += self.L_RIGID * math.sin(theta)

            # --- 2. Soft Section 1 (s2, s3) ---
            # Link4 -> s2
            x += self.L_S_START * math.cos(theta)
            y += self.L_S_START * math.sin(theta)
            theta += s2
            
            # s2 -> s3
            x += self.L_S_MID * math.cos(theta)
            y += self.L_S_MID * math.sin(theta)
            theta += s3
            
            # --- 3. Connection 1 (s3 -> s5) ---
            # Gimbal等を含む剛体区間
            x += self.L_CONN * math.cos(theta)
            y += self.L_CONN * math.sin(theta)
            theta += s5
            
            # --- 4. Soft Section 2 (s5, s6) ---
            # s5 -> s6
            x += self.L_S_MID * math.cos(theta)
            y += self.L_S_MID * math.sin(theta)
            theta += s6
            
            # --- 5. Connection 2 (s6 -> s8) ---
            # Gimbal等を含む剛体区間
            x += self.L_CONN * math.cos(theta)
            y += self.L_CONN * math.sin(theta)
            theta += s8
            
            # --- 6. Soft Section 3 (s8, s9) ---
            # s8 -> s9
            x += self.L_S_MID * math.cos(theta)
            y += self.L_S_MID * math.sin(theta)
            theta += s9

            # --- 7. End Segment (s9 -> Tip) ---
            x += self.L_END * math.cos(theta)
            y += self.L_END * math.sin(theta)
            
            # --- 誤差計算 ---
            pos_error = x**2 + y**2
            ang_error = (theta - 2 * math.pi)**2
            
            # 正則化項 (6変数分)
            reg = 0.001 * (s2**2 + s3**2 + s5**2 + s6**2 + s8**2 + s9**2)
            
            return pos_error + 2.0 * ang_error + reg

        # 最適化実行
        x0 = self.last_sol
        bounds = [(-np.pi, np.pi) for _ in range(6)]
        
        res = minimize(objective_func, x0, method='SLSQP', bounds=bounds, tol=1e-4)
        
        if res.success:
            self.last_sol = res.x
            return res.x.tolist()
        else:
            return self.last_sol

def main():
    try:
        DeltaSoftLinkSolver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()