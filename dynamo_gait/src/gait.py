#!/usr/bin/env python
import numpy as np
from std_msgs.msg import Float64
from math import *
import rospy
import actionlib
from kinematics import dynamo_kinematics
from dynamo_msgs.msg import walk_actionAction, walk_actionGoal, walk_actionResult, walk_actionFeedback
from sensor_msgs.msg import Joy,JointState

pubLF1 = rospy.Publisher('/dynamo/LFJ1_position_controller/command', Float64, queue_size=10)
pubLF2 = rospy.Publisher('/dynamo/LFJ2_position_controller/command', Float64, queue_size=10)
pubLF3 = rospy.Publisher('/dynamo/LFJ3_position_controller/command', Float64, queue_size=10)
pubLB1 = rospy.Publisher('/dynamo/LBJ1_position_controller/command', Float64, queue_size=10)
pubLB2 = rospy.Publisher('/dynamo/LBJ2_position_controller/command', Float64, queue_size=10)
pubLB3 = rospy.Publisher('/dynamo/LBJ3_position_controller/command', Float64, queue_size=10)
pubRF1 = rospy.Publisher('/dynamo/RFJ1_position_controller/command', Float64, queue_size=10)
pubRF2 = rospy.Publisher('/dynamo/RFJ2_position_controller/command', Float64, queue_size=10)
pubRF3 = rospy.Publisher('/dynamo/RFJ3_position_controller/command', Float64, queue_size=10)
pubRB1 = rospy.Publisher('/dynamo/RBJ1_position_controller/command', Float64, queue_size=10)
pubRB2 = rospy.Publisher('/dynamo/RBJ2_position_controller/command', Float64, queue_size=10)
pubRB3 = rospy.Publisher('/dynamo/RBJ3_position_controller/command', Float64, queue_size=10)

M=np.array([[   -8,     18, -11,      0, 1],
            [   -8,     14,  -5,      0, 0],
            [   16,    -32,  16,      0, 0],
            [   -1,    2.5,  -2,    0.5, 0],
            [    2,     -3,   1,      0, 0]])

class gait:
    DC1 = 1
    DC2 = 0
    halt = 1
    rotate = 0
    swing = 1
    jnt = []

    def __init__(self,stepLength = 0.1,stepHeight = 0.02,height =0.17,velocity =0.2):
        self.stepLength,self.stepHeight,self.height,self.velocity = stepLength,stepHeight,height,velocity

    def joystick_callback(self,joy_status):
        right = -joy_status.axes[0]
        front = joy_status.axes[1]
        r = -1*joy_status.buttons[4] + joy_status.buttons[5]
        if abs(r):
            self.rotate = r
            self.halt = 0
        elif right !=0 or front !=0:
            theta = np.arcsin(front/sqrt(right**2+front**2))
            self.halt = 0
            self.rotate = 0
            self.DC1 = round(np.sin(theta),3)
            self.DC2 = -1*round(np.sign(right)*np.cos(theta),3)
        else:
            self.halt = 1
            self.rotate = 0
            self.DC1 = 0
            self.DC2 = 0
    
    def joint_state_callback(self,joint_state):
        self.jnt = joint_state.position

    def point_matrix(self,P1,right = 1):
        P2 = np.array([(right*self.stepLength/2)*self.DC2-0.055, P1[1] ,right*(self.stepLength/2)*self.DC1])
        P3 = np.array([(P1[0]+P2[0])/2,-(self.height-self.stepHeight), (P1[2]+P2[2])/2 ])
        T1 = np.array([right*self.stepLength*self.DC2,0,-right*self.stepLength*self.DC1])
        T2 = np.array([right*self.stepLength*self.DC2,0,-right*self.stepLength*self.DC1])
        return np.array([P1,P2,P3,T1,T2])
    
    def walk(self):
        while self.jnt == []:
            pass
        kinem = dynamo_kinematics()
        stance_current = kinem.leg_fk(self.jnt[6],self.jnt[7],self.jnt[8])#right front joint
        swing_current = kinem.leg_fk(self.jnt[9],self.jnt[10],self.jnt[11])#right back joint
        swing_current = np.array([-0.055,-0.17,0])
        stance_current = np.array([-0.055,-0.17,0])
        stance_current[1] = -self.height
        swing_current[1]  = -self.height
        while not rospy.is_shutdown():
            T = self.stepLength/self.velocity
            if not self.halt and not abs(self.rotate):
                    swing_matrix = self.point_matrix(swing_current)
                    stance_end_point = np.array([-self.DC2*(self.stepLength/2) -0.055,-self.height,-self.DC1*(self.stepLength/2)])
                    start = rospy.get_time()
                    t = 0
                    while(t<=T):
                        u = t/T
                        u = round(u,4)
                        if u>0.96:
                            u =1
                        U = np.array([u**4,u**3,u**2,u,1])
                        B = np.dot(M,U)
                        swing_pnt = np.dot(B,swing_matrix)
                        stance_pnt = stance_current*(1-u) + stance_end_point*u
                        swing_j1,swing_j2,swing_j3 = kinem.leg_Ik(swing_pnt[0],swing_pnt[1],swing_pnt[2])
                        stance_j1,stance_j2,stance_j3 = kinem.leg_Ik(stance_pnt[0],stance_pnt[1],stance_pnt[2])
                        if self.swing:
                            pubLF1.publish(swing_j1)
                            pubLF2.publish(swing_j2)
                            pubLF3.publish(swing_j3)
                            pubRB1.publish(swing_j1)
                            pubRB2.publish(swing_j2)
                            pubRB3.publish(swing_j3)
                            pubLB1.publish(stance_j1)
                            pubLB2.publish(stance_j2)
                            pubLB3.publish(stance_j3)
                            pubRF1.publish(stance_j1)
                            pubRF2.publish(stance_j2)
                            pubRF3.publish(stance_j3)
                        else:
                            pubLB1.publish(swing_j1)
                            pubLB2.publish(swing_j2)
                            pubLB3.publish(swing_j3)
                            pubRF1.publish(swing_j1)
                            pubRF2.publish(swing_j2)
                            pubRF3.publish(swing_j3)
                            pubLF1.publish(stance_j1)
                            pubLF2.publish(stance_j2)
                            pubLF3.publish(stance_j3)
                            pubRB1.publish(stance_j1)
                            pubRB2.publish(stance_j2)
                            pubRB3.publish(stance_j3)
                        t = rospy.get_time() - start
                    swing_current = stance_pnt
                    stance_current = swing_pnt
                    self.swing = 1-self.swing
if __name__ == "__main__":
    dynamo_gait = gait()
    rospy.init_node('Walk_control_server')
    rospy.Subscriber("joy",Joy, dynamo_gait.joystick_callback)
    rospy.Subscriber("dynamo/joint_states",JointState, dynamo_gait.joint_state_callback)
    dynamo_gait.walk()