#!/usr/bin/env python
import numpy as np
from std_msgs.msg import Float64
import rospy
from kinematics import leg_ik
n = 12
comb = []
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
rospy.init_node('joint_controller', anonymous=True)
for i in range(n):
    temp_comb = np.math.factorial(n-1)/(np.math.factorial(i)*np.math.factorial(n-i-1))
    comb.append(temp_comb)
def bezier(Points,t):
    i1 = np.arange(n)
    i2 = np.arange(n)[::-1]
    teq = np.power(t, i1)*np.power((1-t),i2)*comb
    value = [np.dot(Points[0],teq),np.dot(Points[1],teq),np.dot(Points[2],teq)]
    return value
    
def stance(Points,t):
    value = [(1-t)*Points[0][0] + t*Points[0][1],(1-t)*Points[1][0] + t*Points[1][1],(1-t)*Points[2][0] + t*Points[2][1]]
    return value
p1 = np.array([[-0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06],
                [-0.240,-0.240,-0.220,-0.220,-0.220,-0.220,-0.220,-0.200,-0.200,-0.200,-0.240,-0.240],
                    [-0.050,-0.060,-0.090,-0.090,-0.090,0,0,0,0.090,0.090,0.06,0.050]])
p2 = np.array([[-0.06,-0.06],
                [-0.240,-0.240],
                [0.050,-0.05]])
T = 0
swing = 0
velocity = 0.2
distance = 0.1
time = distance/velocity
rate = rospy.Rate(300)
prev = rospy.get_time()                  
while True:
    T = (rospy.get_time()-prev)/time
    coord1 = bezier(p1,T)
    coord2 = stance(p2,T)
    try:
        (thetaSW1,thetaSW2,thetaSW3) =  leg_ik(coord1[0],coord1[1],coord1[2])
        (thetaST1,thetaST2,thetaST3) =  leg_ik(coord2[0],coord2[1],coord2[2])
    except:
        pass
    if swing:
        pubLF1.publish(thetaSW1)
        pubLF2.publish(thetaSW2)
        pubLF3.publish(thetaSW3)
        pubRB1.publish(thetaSW1)
        pubRB2.publish(thetaSW2)
        pubRB3.publish(thetaSW3)
        pubLB1.publish(thetaST1)
        pubLB2.publish(thetaST2)
        pubLB3.publish(thetaST3)
        pubRF1.publish(thetaST1)
        pubRF2.publish(thetaST2)
        pubRF3.publish(thetaST3)
    else:
        pubLF1.publish(thetaST1)
        pubLF2.publish(thetaST2)
        pubLF3.publish(thetaST3)
        pubRB1.publish(thetaST1)
        pubRB2.publish(thetaST2)
        pubRB3.publish(thetaST3)
        pubLB1.publish(thetaSW1)
        pubLB2.publish(thetaSW2)
        pubLB3.publish(thetaSW3)
        pubRF1.publish(thetaSW1)
        pubRF2.publish(thetaSW2)
        pubRF3.publish(thetaSW3)
    rate.sleep()
    if T>=1:
        prev = rospy.get_time()
        swing = not(swing)