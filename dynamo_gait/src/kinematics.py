#!/usr/bin/env python3
import numpy as np
from math import *

class dynamo_kinematics:
    def __init__(self,l1=0.040,l2=0,l3=0.17,l4=0.17,length=0.270,width=0.150):
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
        theta = pi/2  + theta1 - atan((T+self.l2)/self.l1)
        y = -T*cos(theta)
        x = -T*sin(theta)
        ar = np.array([x,y,z])
        return ar
    def body_IK(self,front,right,height,roll,pitch,yaw):
        R1 = np.array([
            [1, 0, 0, 0], 
            [0, np.cos(roll), -np.sin(roll), 0],
            [0,np.sin(roll),np.cos(roll),0],
            [0,0,0,1]])
        R2 = np.array([
            [np.cos(yaw),0, np.sin(yaw), 0], 
            [0, 1, 0, 0],
            [-np.sin(yaw),0, np.cos(yaw),0],
            [0,0,0,1]])
        R3 = np.array([
            [np.cos(pitch),-np.sin(pitch), 0,0], 
            [np.sin(pitch),np.cos(pitch),0,0],
            [0,0,1,0],
            [0,0,0,1]])
        R = R3@R2@R1
        T = np.array([[0,0,0,front],[0,0,0,-height],[0,0,0,right],[0,0,0,0]])
        Transformation = T + R
        Trb =np.array([
            [np.cos(-pi/2),0,np.sin(-pi/2),self.width/2],
            [0,1,0,0],
            [-np.sin(-pi/2),0,np.cos(-pi/2),self.length/2],
            [0,0,0,1]])@Transformation
        Trf = np.array([
            [np.cos(-pi/2),0,np.sin(-pi/2),self.width/2],
            [0,1,0,0],
            [-np.sin(-pi/2),0,np.cos(-pi/2),-self.length/2],
            [0,0,0,1]])@Transformation
        Tlf = np.array([
            [np.cos(pi/2),0,np.sin(pi/2),self.width/2],
            [0,1,0,0],
            [-np.sin(pi/2),0,np.cos(pi/2),self.length/2],
            [0,0,0,1]])@Transformation
        Tlb =np.array([
            [np.cos(pi/2),0,np.sin(pi/2),self.width/2],
            [0,1,0,0],
            [-np.sin(pi/2),0,np.cos(pi/2),-self.length/2],
            [0,0,0,1]])@Transformation
        return np.array([Trb,Trf,Tlf,Tlb])
    def body_to_leg_IK(self,front =0,right =0,height = 0.15,roll = 0,pitch = 0,yaw =0):
        T = self.body_IK(front,right,height,roll,pitch,yaw)
        Trb,Trf,Tlf,Tlb = T[0],T[1],T[2],T[3]
        LF = (T[2]@np.array([self.length/2, 0,-(self.width/2+self.l1),1]))[:3]
        LB = (T[3]@np.array([-self.length/2,0,-(self.width/2+self.l1),1]))[:3]
        RF = (T[1]@np.array([self.length/2, 0, (self.width/2+self.l1),1]))[:3]
        RB = (T[0]@np.array([-self.length/2,0, (self.width/2+self.l1),1]))[:3]
        J = np.zeros(12)
        J[0],J[1],J[2]  = self.leg_Ik(LF[0],LF[1],LF[2],-1)
        J[3],J[4],J[5]  = self.leg_Ik(LB[0],LB[1],LB[2],-1)
        J[6],J[7],J[8]  = self.leg_Ik(RF[0],RF[1],RF[2],1)
        J[9],J[10],J[11]= self.leg_Ik(RB[0],RB[1],RB[2],1)
        return J,LF,LB,RF,RB
if __name__ == "__main__":
    kinematics = dynamo_kinematics()
    J,LF,LB,RF,RB = kinematics.body_to_leg_IK(0,0,0.15,0,0,0.35)
    print(LF)



