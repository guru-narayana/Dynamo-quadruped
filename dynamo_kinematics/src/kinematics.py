import numpy as np
import time
from math import *
l1 = 0.055
l2 = 0
l3 = 0.107
l4 = 0.130

def leg_ik(x,y,z,right = 1):
    F=sqrt(x**2+y**2-l1**2)
    G=F-l2  
    H=sqrt(G**2+z**2)
    theta1=atan2(-y,x)-atan2(F,-l1)
    D=(H**2-l3**2-l4**2)/(2*l3*l4)
    theta3=atan2(sqrt(1-D**2),D) 
    theta2=atan2(right*z,G)-atan2(l4*sin(theta3),l3+l4*cos(theta3))
    return(theta1,theta2,theta3)

def leg_fk(theta1,theta2,theta3,right = 1):
    p = sqrt(l3**2 + l4**2 - 2*l3*l4*cos(pi-theta3))
    alpha = theta2 + atan2(l4*sin(theta3),l3+l4*cos(theta3))
    L = abs(p*cos(alpha))
    z = right*p*sin(alpha)
    T = sqrt((l2+L)**2 + l1**2)
    theta = pi/2  + theta1 - atan((L+l2)/l1)
    y = -T*cos(theta)
    x = -T*sin(theta)
    ar = np.array([x,y,z])
    return ar
