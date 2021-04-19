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

# Direction cosines for the endpoint direciton
DC1 = 1
DC2 = 0
halt = 1
rotate = 0
swing = 0
jnt = []
