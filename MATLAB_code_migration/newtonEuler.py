#!/usr/bin/env python
import rospy

from numpy import *
from sympy import *
from dh_transform import dhTransform
from forwardKin import *
from scipy.misc import *

pi = numpy.pi

#Testing DH table
dh_table = numpy.array([[pi/6,2,3,0],[pi/4,5,10,pi/2],[pi/2,4,1,pi/2],[0,5,3,0]])
rho = numpy.array([1,1,1,1])

#def newtonEuler()

#dh transformation check
T1 = dhTransform(dh_table[0,0],dh_table[0,1],dh_table[0,2],dh_table[0,3])
print "T1"
print T1

T2 = dhTransform(dh_table[1,0],dh_table[1,1],dh_table[1,2],dh_table[1,3])
print "T2"
print T2


# Compute angular and linear velocity for force component

def getRotMatrix(i,dh_table):
	#get the rotation matrix of ith and ith+1 transformation matrix

	#First, acquire the parameters of each link
	for j in range (i,i+1):
		#for ith link
		theta_i = dh_table[j,0]
		d_i = dh_table[j,1]
		a_i = dh_table[j,2]
		alpha_i = dh_table[j,3]
		#for ith+1 link 
		theta_ip1 = dh_table[j+1,0]
		d_ip1 = dh_table[j+1,1]
		a_ip1 = dh_table[j+1,2]
		alpha_ip1 = dh_table[j+1,3]
	#Get the ration matrices
	T_i = dhTransform(theta_i,d_i,a_i,alpha_i)
	R_i = T_i[0:3,0:3]
	T_ip1 = dhTransform(theta_ip1,d_ip1,a_ip1,alpha_ip1)
	R_ip1 = T_ip1[0:3,0:3]
	return R_i, R_ip1  

#function test
R_i, R_ip1 = getRotMatrix(1,dh_table)
print R_i
print R_ip1

def getPosMatrix(i,dh_table):
	for j in range (i,i+1):
		#for ith link
		theta_i = dh_table[j,0]
		d_i = dh_table[j,1]
		a_i = dh_table[j,2]
		alpha_i = dh_table[j,3]
		#for ith+1 link 
		theta_ip1 = dh_table[j+1,0]
		d_ip1 = dh_table[j+1,1]
		a_ip1 = dh_table[j+1,2]
		alpha_ip1 = dh_table[j+1,3]
	#Get the ration matrices
	T_i = dhTransform(theta_i,d_i,a_i,alpha_i)
	P_i = T_i[0:3,3]
	T_ip1 = dhTransform(theta_ip1,d_ip1,a_ip1,alpha_ip1)
	P_ip1 = T_ip1[0:3,3]
	return P_i, P_ip1 
	
#function test
P_i, P_ip1 = getPosMatrix(1,dh_table)
print P_i
print P_ip1

def angularVelAc(dh_table, dq,ddq, joint_config):
		
	n = len(dh_parameter)
	g = 9.81			#gravitational acceleration (scalar)
	
	w = [[0],[0],[0]]		#ground angular velocity
	dw = [[0],[0],[0]]		#ground angular acceleration
	z = numpy.array[[0],[0],[1]]
	
	if len(joint_config) != len(dh_table):
		print "cannot find solution"
	
	else:
		for i in range (0,n):
			R_i, R_ip1 = getRotMatrix(i,dh_table)
			R = numpy.dot(R_i,R_ip1)
			if joint_config[n] == 0:		#if joint is prismatic
				w_ip1 = numpy.dot(R,w)
				w = w_ip1
				dw_ip1 = numpy.dot(R,dw)
				dw = dw_ip1
			elif joint_config[n] == 1:		#if joint is revolute
				w_ip1 = numpy.dot(R,w)+numpy.dot(q[i],z)
				w = w_ip1
				dw_ip1 = numpy.dot(R,dw) + numpy.cross(numpy.dot(R,w),numpy.dot(dq,z)) + numpy.dot(dq,z)
				dw = dw_ip1
			else:
				print "not a standard joint configuration"
			
	return w, dw
	
#def linearVelAc(dh_table, w):
	







