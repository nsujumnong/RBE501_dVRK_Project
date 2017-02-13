#!/usr/bin/env python
import rospy

from numpy import *
from sympy import *
from dh_transform import dhTransform
from forwardKin import *
from scipy.misc import *

pi = numpy.pi

#Testing parameters and DH table
dh_table = numpy.array([[pi/6,2,3,0],[pi/4,5,10,pi/2],[pi/2,4,1,pi/2],[0,5,3,0]])
rho = numpy.array([1,1,1,1])
dq = numpy.array([1,1,1,1])
ddq = numpy.array([1,1,1,1])
joint_config = numpy.array([1,1,1,1])

#def newtonEuler()

#dh transformation check
#T1 = dhTransform(dh_table[0,0],dh_table[0,1],dh_table[0,2],dh_table[0,3])
#print "T1"
#print T1

#T2 = dhTransform(dh_table[1,0],dh_table[1,1],dh_table[1,2],dh_table[1,3])
#print "T2"
#print T2


# Compute angular and linear velocity for force component

def getRotMatrix(i,dh_table):
	#get the rotation matrix of ith and ith+1 transformation matrix

	#First, acquire the parameters of each link
	for j in range (i-1,i):
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
#R_i, R_ip1 = getRotMatrix(1,dh_table)
#print R_i
#print R_ip1

def getPosMatrix(i,dh_table):
	for j in range (i-1,i):
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
#P_i, P_ip1 = getPosMatrix(1,dh_table)
#print P_i
#print P_ip1

def velAndAccel(dh_table, dq,ddq, joint_config):
	#compute rotational and linear velocity and acceleration
	n = len(dh_table)
	g = 9.81			#gravitational acceleration (scalar)
	
	w_i = [[0],[0],[0]]		#ground angular velocity
	dw_i = [[0],[0],[0]]		#ground angular acceleration
	v_i = [[0],[0],[0]]		#initial linear velocity
	dv_i = [[0],[0],[-g]]		#initial linear acceleration (gravitational acceleration)
	z = [[0],[0],[1]]
	
	if len(joint_config) != len(dh_table):
		print "cannot find solution"
	
	else:
		for i in range (0,n):
			R_i, R_ip1 = getRotMatrix(i,dh_table)
			R = numpy.dot(R_i,R_ip1)
			P_i, P_ip1 = getPosMatrix(i,dh_table)
			#print "P_", i+1
			#print P_i
			#print "P_", i+2
			#print P_ip1
				
			P = numpy.dot(P_i,P_ip1)
			#print "P_",i+1,"_",i+2
			#print P
			
			if joint_config[i] == 0:		#if joint is prismatic
				w_ip1 = numpy.dot(R,w_i)
				w_i = w_ip1
				dw_ip1 = numpy.dot(R,dw_i)
				dw_i = dw_ip1
			elif joint_config[i] == 1:		#if joint is revolute
				#angular velocity
				w_ip1 = numpy.dot(R,w_i)+numpy.dot(dq[i],z)
				w_i = w_ip1
				#print "w_",i+1
				#w_i_T = w_i.T
				#print w_i_T
				#print w_i_T.shape
				#print "dw_", i
				#print dw_i
				#angular acceleration
				dw_ip1 = numpy.dot(R,dw_i) + numpy.cross(numpy.dot(R,w_i).T,numpy.dot(dq[i],z).T) + numpy.dot(ddq[i],z)
				dw_i = dw_ip1
				
				#linear velocity
				#comp1 = numpy.cross(w_i.T,P.T)
				#comp2 = numpy.cross(w_i.T,P.T)+numpy.cross(w_i.T,comp1)+dv_i
				#dv_ip1 = numpy.dot(R,comp2)
				#dv_i = dv_ip1
			else:
				print "not a standard joint configuration"
	
	return w_i		
	#return w_i, dw_i, dv_i
	
#algorithm test
#w_1, dw_1, dv_1 = velAndAccel(dh_table,dq,ddq,joint_config)
w_1 = velAndAccel(dh_table,dq,ddq,joint_config)

#def linearVelAc(dh_table, joint_config):
#	n = len(dh_table)
#	g = 9.81
#	
#	w = [[0],[0],[0]]		#ground angular velocity
#	dw = [[0],[0],[0]]		#ground angular acceleration
#	v = [[0],[0],[0]]
#	dv = [[0],[0],[-g]]
#	
#	for i in range (0,n):
#		R_i, R_ip1 = getRotMatrix(i,dh_table)
#		R = numpy.dot(R_i,R_ip1)
#		P_i, P_ip1 = getPosMatrix(i,dh_table)
#		P = numpy.dot(P_i,P_ip1)
#		# break down the linear velocity equation into several components
#		#first... the w_i x P
#		comp1 = numpy.cross(w,P)
#		#second... dw_i x P + w_i x (comp1) + dv
#		comp2 = numpy.cross(w,P)+numpy.cross(w,comp1)+dv_i
#		#finally, get the dot product of R and the components
#		dv_ip1 = numpy.dot(R,comp2)
#		dv_i = dv_ip1
#	return dv_i






