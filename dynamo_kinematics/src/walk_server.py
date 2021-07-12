#!/usr/bin/env python
import numpy as np
from std_msgs.msg import Float64
from math import *
import rospy
import actionlib
from kinematics import leg_ik,leg_fk
from dynamo_msgs.msg import walk_actionAction, walk_actionGoal, walk_actionResult, walk_actionFeedback
from sensor_msgs.msg import Joy,JointState

stepLength = 0.1
stepHeight = 0.1
Height = 0.25
# Direction cosines for the endpoint direciton
DC1 = 1
DC2 = 0
halt = 1
rotate = 0
swing = 0
jnt = []
# numerical matrix for blending function
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

# calculates direction cosines from joystick
def joystick_callback(joy_status):
    global DC1,DC2,rotate,halt
    right = -joy_status.axes[0]
    front = joy_status.axes[1]
    r = -1*joy_status.buttons[4] + joy_status.buttons[5]
    if abs(r):
        rotate = r
        halt = 0
    elif right !=0 or front !=0:
        theta = np.arcsin(front/sqrt(right**2+front**2))
        halt = 0
        rotate = 0
        DC1 = round(np.sin(theta),3)
        DC2 = round(np.sign(right)*np.cos(theta),3)
    else:
        halt = 1
        rotate = 0
        DC1 = 0
        DC2 = 0
def joint_state_callback(joint_state):
    global jnt
    jnt = joint_state.position

def point_matrix(P1,DC1,DC2,right = 1):
    P2 = np.array([right*(stepLength/2)*DC2-0.07, P1[1] ,right*(stepLength/2)*DC1])
    P3 = np.array([(P1[0]+P2[0])/2,-(Height-stepHeight), (P1[2]+P2[2])/2 ])
    T1 = np.array([right*stepLength*DC2,0,-right*stepLength*DC1])
    T2 = np.array([right*stepLength*DC2,0,-right*stepLength*DC1])
    return np.array([P1,P2,P3,T1,T2])

def execute(set_type = 1):
    global DC1,DC2,halt,rotate,swing,jnt,stepLength,stepHeight,Height
    velocity = 0.2
    stepLength = 0.1
    stepHeight = 0.03
    T = stepLength/velocity
    while jnt == []:
        pass
    LF = leg_fk(jnt[0],jnt[1],jnt[2],-1)
    LB = leg_fk(jnt[3],jnt[4],jnt[5],-1)
    RF = leg_fk(jnt[6],jnt[7],jnt[8],1)
    RB = leg_fk(jnt[9],jnt[10],jnt[11],1)
    LF[1] = -Height
    RF[1] = -Height
    RB[1] = -Height
    LB[1] = -Height
    print(LF,LB,RF)
    while not rospy.is_shutdown():
        if jnt != []:
            if not halt and not abs(rotate):
                if swing:
                    swing_lf_points = point_matrix(LF,DC1,DC2,-1)
                    swing_rb_points = point_matrix(RB,DC1,DC2,1)
                    rf_end_point = np.array([-DC2*(stepLength/2) -0.07,RF[1],-DC1*(stepLength/2)])
                    print(rf_end_point,RF)
                    lb_end_point = np.array([DC2*(stepLength/2) -0.07,LB[1],DC1*(stepLength/2)])
                    start = rospy.get_time()
                    t = 0
                    while (t <T):
                        u = t/T
                        u = round(u,4)
                        if u>0.99:
                            u =1
                        U = np.array([u**4,u**3,u**2,u,1])
                        B = np.dot(M,U)                        
                        p_lf = np.dot(B,swing_lf_points)
                        p_rb = np.dot(B,swing_rb_points)
                        p_rf = RF*(1-u) + rf_end_point*u
                        p_lb = LB*(1-u) + lb_end_point*u
                        LF_J1,LF_J2,LF_J3 = leg_ik(p_lf[0],p_lf[1],p_lf[2],-1)
                        RF_J1,RF_J2,RF_J3 = leg_ik(p_rf[0],p_rf[1],p_rf[2])
                        LB_J1,LB_J2,LB_J3 = leg_ik(p_lb[0],p_lb[1],p_lb[2],-1)
                        RB_J1,RB_J2,RB_J3 = leg_ik(p_rb[0],p_rb[1],p_rb[2])
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
                        swing = 0
                        t = rospy.get_time() - start
                else:
                    swing_rf_points = point_matrix(RF,DC1,DC2,1)
                    swing_lb_points = point_matrix(LB,DC1,DC2,-1)
                    rb_end_point = np.array([-DC2*(stepLength/2)-0.07,RB[1],-DC1*(stepLength/2)])
                    lf_end_point = np.array([DC2*(stepLength/2)-0.07,LF[1],DC1*(stepLength/2)])
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
                        LF_J1,LF_J2,LF_J3 = leg_ik(p_lf[0],p_lf[1],p_lf[2],-1)
                        RF_J1,RF_J2,RF_J3 = leg_ik(p_rf[0],p_rf[1],p_rf[2])
                        LB_J1,LB_J2,LB_J3 = leg_ik(p_lb[0],p_lb[1],p_lb[2],-1)
                        RB_J1,RB_J2,RB_J3 = leg_ik(p_rb[0],p_rb[1],p_rb[2])
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
                        swing = 1
                        t = rospy.get_time() - start
                LF = p_lf
                LF[1] = -Height
                LB = p_lb
                LB[1] = -Height
                RF = p_rf
                RF[1] = -Height
                RB = p_rb
                RB[1] = -Height
rospy.init_node('Walk_control_server')
rospy.Subscriber("joy",Joy, joystick_callback)
rospy.Subscriber("dynamo/joint_states",JointState, joint_state_callback)
execute()