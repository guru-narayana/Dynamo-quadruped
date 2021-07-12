#! /usr/bin/env python
import rospy
import actionlib
import time
from dynamo_msgs.msg import Dynamo_motionAction, Dynamo_motionGoal, Dynamo_motionResult, Dynamo_motionFeedback
from sensor_msgs.msg import Joy,JointState
from math import *

server = 1
initial = 1

height = 0.22
front = 0
right = 0

def joystick_callback(joy):
    global server
    server = min(joy.buttons[0]*1 + joy.buttons[1]*2,2)

rospy.init_node('arm_initiation_node')
rospy.Subscriber("joy",Joy, joystick_callback)

client1 = actionlib.SimpleActionClient('gait_action', Dynamo_motionAction)
client2 = actionlib.SimpleActionClient('rpy_action', Dynamo_motionAction)
client1.wait_for_server()
client2.wait_for_server()

while not rospy.is_shutdown():
    start_status = Dynamo_motionGoal()
    start_status.height = height
    start_status.front = front
    start_status.right = right
    if server == 1:
        if initial == 0:
            client2.cancel_goal()

