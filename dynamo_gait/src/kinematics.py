import numpy as np
from math import *

class dynamo_kinematics:
    def __init__(self,l1=0.055,l2=0,l3=0.107,l4=0.130,length=0.186,width=0.078):
        self.l1,self.l2,self.l3,self.l4,self.length,self.width = l1,l2,l3,l4,length,width
    def leg_Ik(self,x,y,z,right = 1):
        F=sqrt(x**2+y**2- self.l1**2)
        G=F-self.l2  
        H=sqrt(G**2+z**2)
        theta1=right*(atan2(-y,x)-atan2(F,-self.l1))
        D=(H**2-self.l3**2-self.l4**2)/(2*self.l3*self.l4)
        theta3=atan2(sqrt(1-D**2),D)
        theta2=atan2(right*z,G)-atan2(self.l4*sin(theta3),self.l3+self.l4*cos(theta3))
        return(theta1,theta2,theta3)
    def leg_fk(self,theta1,theta2,theta3,right = 1):
        p = sqrt(self.l3**2 + self.l4**2 - 2*self.l3*self.l4*cos(pi-theta3))
        alpha = theta2 + atan2(self.l4*sin(theta3),self.l3+self.l4*cos(theta3))
        L = abs(p*cos(alpha))
        z = right*p*sin(alpha)
        T = sqrt((self.l2+L)**2 + self.l1**2)
        theta = pi/2  + theta1 - atan((L+self.l2)/self.l1)
        y = -T*cos(theta)
        x = -T*sin(theta)
        ar = np.array([x,y,z])
        return ar
    def body_IK(self):
        pass

if __name__ == "__main__":
    kinematics = dynamo_kinematics()
    t1,t2,t3 = kinematics.leg_Ik(-0.08,-0.13,0)
    print(round(t1,2))
    print(round(t2,2))
    print(round(t3,2))
