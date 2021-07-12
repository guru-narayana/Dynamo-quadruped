#!/usr/bin/env python3
import numpy as np
from std_msgs.msg import Float64
from math import *
import rospy
import actionlib
from kinematics import dynamo_kinematics
from dynamo_msgs.msg import walk_actionAction, walk_actionGoal, walk_actionResult, walk_actionFeedback
from sensor_msgs.msg import Joy,JointState

#pos update frequency
frequency = 30

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
class rpy:
    #limits of motions
    max_pitch = (pi/180)*30
    max_roll = (pi/180)*30
    max_yaw = (pi/180)*45
    max_height,min_height = 0.26,0.19
    max_front,min_front = 0.06,-0.06
    Max_angular_step = (pi/180)*30

    #veloctiy
    roll_vel = (pi/180)*30/frequency
    pitch_vel = (pi/180)*30/frequency
    yaw_vel = (pi/180)*30/frequency
    vertical_vel = 0.02/frequency
    horizontal_vel = 0.02/frequency

    #max_step
    horizontal_step = 0.005
    vertical_step = 0.01
    
    #goal_status
    roll = 0
    pitch = 0
    yaw = 0
    height = 0.22
    front = 0

    #current status
    current_roll = 0
    current_pitch = 0
    current_yaw = 0
    current_height = 0.22
    current_front = 0
    
    #joystick callback
    def joystick_callback(self,joy_status):
        self.roll = joy_status.axes[0]*self.max_roll
        self.pitch = joy_status.axes[1]*self.max_pitch
        self.yaw = -joy_status.axes[2]*self.max_yaw
        self.height = max(min(self.height - joy_status.axes[7]*self.vertical_step,self.max_height),self.min_height)
        self.front =  max(min(self.front - joy_status.axes[6]*self.horizontal_step,self.max_front),self.min_front)
    def execute_rpy(self):
        kinem = dynamo_kinematics()
        while not rospy.is_shutdown():
            self.current_roll += max(min(self.roll_vel,(self.roll-self.current_roll)),-self.roll_vel)
            self.current_pitch += max(min(self.pitch_vel,(self.pitch-self.current_pitch)),-self.pitch_vel)
            self.current_yaw += max(min(self.yaw_vel,(self.yaw-self.current_yaw)),-self.yaw_vel)
            self.current_height += max(min(self.vertical_vel,(self.height-self.current_height)),-self.vertical_vel)
            self.current_front += max(min(self.horizontal_vel,(self.front-self.current_front)),-self.horizontal_vel)
            J,LF,LB,RF,RB = kinem.body_to_leg_IK(self.current_front,0,self.current_height,self.current_roll,self.current_pitch,self.current_yaw)
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
            rate.sleep()
if __name__ == "__main__":
    dynamo_rpy = rpy()
    rospy.init_node('RPY_motion_server')
    rospy.Subscriber("joy",Joy, dynamo_rpy.joystick_callback)
    rate = rospy.Rate(frequency)
    dynamo_rpy.execute_rpy()