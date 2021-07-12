#!/usr/bin/env python3
import numpy as np
from std_msgs.msg import Float64
from math import *
import rospy
import actionlib
from kinematics import dynamo_kinematics
from dynamo_msgs.msg import Dynamo_motionAction, Dynamo_motionGoal, Dynamo_motionResult, Dynamo_motionFeedback
from sensor_msgs.msg import Joy,JointState
from dynamo_msgs.msg import Param

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
    # direction cosines
    DC1 = 1
    DC2 = 0
    halt = 1
    rotate = 0

    #gait params
    swing = 1
    stepLength = 0.1
    angular_step = (pi/180)*15
    velocity =0.16
    omega = 0.3
    stepHeight = 0.03
    rotate_start = 1

    # basic robot parameters
    height =0.22
    front  = 0
    right = 0
    
    jnt = []
    # joystic call back to determine the direction cosine values
    def joystick_callback(self,joy_status):
        right = -joy_status.axes[0]
        front = joy_status.axes[1]
        r = -1*joy_status.buttons[4] + joy_status.buttons[5]
        if abs(r):
            self.rotate = r
            self.halt = 0
            self.DC1 = 0
            self.DC2 = 0
        elif right !=0 or front !=0:
            theta = np.arcsin(front/sqrt(right**2+front**2))
            self.halt = 0
            self.rotate = 0
            self.DC1 = round(np.sin(theta),3)
            self.DC2 = round(np.sign(right)*np.cos(theta),3)
        else:
            self.halt = 1
            self.rotate = 0
            self.DC1 = 0
            self.DC2 = 0

    # joint state subscriber to determine the current joint states which later be used by forward kinematics
    def joint_state_callback(self,joint_state):
        self.jnt = joint_state.position
    
    #blender matrix generater for determining the swing phase trajectory
    def point_matrix(self,P1,right = 1):
        P2 = np.array([(-1*right*self.stepLength/2)*self.DC2-0.040, P1[1] ,right*(self.stepLength/2)*self.DC1])
        P3 = np.array([(P1[0]+P2[0])/2,-(self.height-self.stepHeight), (P1[2]+P2[2])/2 ])
        T1 = np.array([right*self.stepLength*self.DC2,0,-right*self.stepLength*self.DC1])
        T2 = np.array([right*self.stepLength*self.DC2,0,-right*self.stepLength*self.DC1])
        return np.array([P1,P2,P3,T1,T2])
    
    def walk(self):
        while self.jnt == []:
            pass
        kinem = dynamo_kinematics()
        LF = kinem.leg_fk(self.jnt[0],self.jnt[1],self.jnt[2],-1)
        LB = kinem.leg_fk(self.jnt[3],self.jnt[4],self.jnt[5],-1)
        RF = kinem.leg_fk(self.jnt[6],self.jnt[7],self.jnt[8],1)
        RB = kinem.leg_fk(self.jnt[9],self.jnt[10],self.jnt[11],1)
        LF = np.array([-0.055,-self.height,0])
        LB = np.array([-0.055,-self.height,0])
        RF = np.array([-0.055,-self.height,0])
        RB = np.array([-0.055,-self.height,0])
        LF[1] = -self.height
        RF[1] = -self.height
        RB[1] = -self.height
        LB[1] = -self.height
        while not rospy.is_shutdown():
            if not self.halt and not abs(self.rotate):
                T = self.stepLength/self.velocity
                self.rotate_start =1
                if self.swing:
                    swing_lf_points = self.point_matrix(LF,-1)
                    swing_rb_points = self.point_matrix(RB,1)
                    rf_end_point = np.array([self.DC2*(self.stepLength/2) -0.040,RF[1],-self.DC1*(self.stepLength/2)])
                    lb_end_point = np.array([-self.DC2*(self.stepLength/2) -0.040,LB[1],self.DC1*(self.stepLength/2)])
                    start = rospy.get_time()
                    t = 0
                    while(t<T):
                        u = t/T
                        u = round(u,4)
                        U = np.array([u**4,u**3,u**2,u,1])
                        B = np.dot(M,U)                        
                        p_lf = np.dot(B,swing_lf_points)
                        p_rb = np.dot(B,swing_rb_points)
                        p_rf = RF*(1-u) + rf_end_point*u
                        p_lb = LB*(1-u) + lb_end_point*u
                        LF_J1,LF_J2,LF_J3 = kinem.leg_Ik(p_lf[0],p_lf[1],p_lf[2],-1)
                        RF_J1,RF_J2,RF_J3 = kinem.leg_Ik(p_rf[0],p_rf[1],p_rf[2])
                        LB_J1,LB_J2,LB_J3 = kinem.leg_Ik(p_lb[0],p_lb[1],p_lb[2],-1)
                        RB_J1,RB_J2,RB_J3 = kinem.leg_Ik(p_rb[0],p_rb[1],p_rb[2])
                        pubLF1.publish(LF_J1)
                        pubLF2.publish(LF_J2)
                        pubLF3.publish(LF_J3)
                        pubRB1.publish(RB_J1)
                        pubRB2.publish(RB_J2)
                        pubRB3.publish(RB_J3)
                        pubLB1.publish(LB_J1)
                        pubLB2.publish(LB_J2)
                        pubLB3.publish(LB_J3)
                        pubRF1.publish(RF_J1)
                        pubRF2.publish(RF_J2)
                        pubRF3.publish(RF_J3)
                        t = rospy.get_time() - start
                    self.swing = 0
                else:
                    swing_rf_points = self.point_matrix(RF,1)
                    swing_lb_points = self.point_matrix(LB,-1)
                    rb_end_point = np.array([self.DC2*(self.stepLength/2)-0.040,RB[1],-self.DC1*(self.stepLength/2)])
                    lf_end_point = np.array([-self.DC2*(self.stepLength/2)-0.040,LF[1],self.DC1*(self.stepLength/2)])
                    start = rospy.get_time()
                    t = 0
                    while (t < T):
                        t = rospy.get_time() - start
                        u = t/T
                        U = np.array([u**4,u**3,u**2,u,1]).T
                        B = np.dot(M,U)                        
                        p_lb = np.dot(B,swing_lb_points)
                        p_rf = np.dot(B,swing_rf_points)
                        p_rb = RB*(1-u) + rb_end_point*u
                        p_lf = LF*(1-u) + lf_end_point*u
                        LF_J1,LF_J2,LF_J3 = kinem.leg_Ik(p_lf[0],p_lf[1],p_lf[2],-1)
                        RF_J1,RF_J2,RF_J3 = kinem.leg_Ik(p_rf[0],p_rf[1],p_rf[2])
                        LB_J1,LB_J2,LB_J3 = kinem.leg_Ik(p_lb[0],p_lb[1],p_lb[2],-1)
                        RB_J1,RB_J2,RB_J3 = kinem.leg_Ik(p_rb[0],p_rb[1],p_rb[2])
                        pubLF1.publish(LF_J1)
                        pubLF2.publish(LF_J2)
                        pubLF3.publish(LF_J3)
                        pubRB1.publish(RB_J1)
                        pubRB2.publish(RB_J2)
                        pubRB3.publish(RB_J3)
                        pubLB1.publish(LB_J1)
                        pubLB2.publish(LB_J2)
                        pubLB3.publish(LB_J3)
                        pubRF1.publish(RF_J1)
                        pubRF2.publish(RF_J2)
                        pubRF3.publish(RF_J3)
                        t = rospy.get_time() - start
                    self.swing = 1
                LF = p_lf
                LF[1] = -self.height
                LB = p_lb
                LB[1] = -self.height
                RF = p_rf
                RF[1] = -self.height
                RB = p_rb
                RB[1] = -self.height
            elif abs(self.rotate):
                T = self.angular_step/self.omega
                start = rospy.get_time()
                t = 0
                if self.rotate_start:
                    LF[1] = -self.height
                    LB[1] = -self.height
                    RF[1] = -self.height
                    RB[1] = -self.height
                    swing_lf_points = self.point_matrix(LF,-1)
                    swing_rb_points = self.point_matrix(RB,1)
                    start = rospy.get_time()
                    t = 0
                    while(t<T):
                        u = t/T
                        u = round(u,4)
                        U = np.array([u**4,u**3,u**2,u,1])
                        B = np.dot(M,U)                        
                        p_lf = np.dot(B,swing_lf_points)
                        p_rb = np.dot(B,swing_rb_points)
                        LF_J1,LF_J2,LF_J3 = kinem.leg_Ik(p_lf[0],p_lf[1],p_lf[2],-1)
                        RB_J1,RB_J2,RB_J3 = kinem.leg_Ik(p_rb[0],p_rb[1],p_rb[2])
                        pubLF1.publish(LF_J1)
                        pubLF2.publish(LF_J2)
                        pubLF3.publish(LF_J3)
                        pubRB1.publish(RB_J1)
                        pubRB2.publish(RB_J2)
                        pubRB3.publish(RB_J3)
                        t = rospy.get_time() - start
                    swing_rf_points = self.point_matrix(RF,1)
                    swing_lb_points = self.point_matrix(LB,-1)
                    start = rospy.get_time()
                    t = 0
                    while (t < T):
                        t = rospy.get_time() - start
                        u = t/T
                        U = np.array([u**4,u**3,u**2,u,1]).T
                        B = np.dot(M,U)                        
                        p_lb = np.dot(B,swing_lb_points)
                        p_rf = np.dot(B,swing_rf_points)
                        RF_J1,RF_J2,RF_J3 = kinem.leg_Ik(p_rf[0],p_rf[1],p_rf[2])
                        LB_J1,LB_J2,LB_J3 = kinem.leg_Ik(p_lb[0],p_lb[1],p_lb[2],-1)
                        pubLB1.publish(LB_J1)
                        pubLB2.publish(LB_J2)
                        pubLB3.publish(LB_J3)
                        pubRF1.publish(RF_J1)
                        pubRF2.publish(RF_J2)
                        pubRF3.publish(RF_J3)
                    self.rotate_start =0

                while(t<=T):
                    angle = (t/T)*self.angular_step
                    J,LF,LB,RF,RB = kinem.body_to_leg_IK(0,0,self.height,0,0,self.rotate*angle)
                    pubLF1.publish(J[0])
                    pubLF2.publish(J[1])
                    pubLF3.publish(J[2])
                    pubLB1.publish(J[3])
                    pubLB2.publish(J[4])
                    pubLB3.publish(J[5])
                    pubRF1.publish(J[6])
                    pubRF2.publish(J[7])
                    pubRF3.publish(J[8])
                    pubRB1.publish(J[9])
                    pubRB2.publish(J[10])
                    pubRB3.publish(J[11])
                    t = rospy.get_time() - start
                LF[1] = -self.height
                LB[1] = -self.height
                RF[1] = -self.height
                RB[1] = -self.height
                swing_lf_points = self.point_matrix(LF,-1)
                swing_rb_points = self.point_matrix(RB,1)
                start = rospy.get_time()
                t = 0
                T = T/2
                while(t<T):
                    u = t/T
                    u = round(u,4)
                    U = np.array([u**4,u**3,u**2,u,1])
                    B = np.dot(M,U)                        
                    p_lf = np.dot(B,swing_lf_points)
                    p_rb = np.dot(B,swing_rb_points)
                    LF_J1,LF_J2,LF_J3 = kinem.leg_Ik(p_lf[0],p_lf[1],p_lf[2],-1)
                    RB_J1,RB_J2,RB_J3 = kinem.leg_Ik(p_rb[0],p_rb[1],p_rb[2])
                    pubLF1.publish(LF_J1)
                    pubLF2.publish(LF_J2)
                    pubLF3.publish(LF_J3)
                    pubRB1.publish(RB_J1)
                    pubRB2.publish(RB_J2)
                    pubRB3.publish(RB_J3)
                    t = rospy.get_time() - start
                swing_rf_points = self.point_matrix(RF,1)
                swing_lb_points = self.point_matrix(LB,-1)
                start = rospy.get_time()
                t = 0

                while (t < T):
                    t = rospy.get_time() - start
                    u = t/T
                    U = np.array([u**4,u**3,u**2,u,1]).T
                    B = np.dot(M,U)                        
                    p_lb = np.dot(B,swing_lb_points)
                    p_rf = np.dot(B,swing_rf_points)
                    RF_J1,RF_J2,RF_J3 = kinem.leg_Ik(p_rf[0],p_rf[1],p_rf[2])
                    LB_J1,LB_J2,LB_J3 = kinem.leg_Ik(p_lb[0],p_lb[1],p_lb[2],-1)
                    pubLB1.publish(LB_J1)
                    pubLB2.publish(LB_J2)
                    pubLB3.publish(LB_J3)
                    pubRF1.publish(RF_J1)
                    pubRF2.publish(RF_J2)
                    pubRF3.publish(RF_J3)
if __name__ == "__main__":
    dynamo_gait = gait()
    rospy.init_node('Walk_control_server')
    rospy.Subscriber("joy",Joy, dynamo_gait.joystick_callback)
    rospy.Subscriber("dynamo/joint_states",JointState, dynamo_gait.joint_state_callback)
    dynamo_gait.walk()