#!/usr/bin/env python
import rospy

from numpy import *
from sympy import *
from dh_transform import dhTransform
from forwardKin import *
from scipy.misc import *
from getPosMatrix import getPosMatrix
from getRotMatrix import getRotMatrix

pi = numpy.pi

#Testing parameters and DH table
dh_table = numpy.array([[pi/6,2,3,0],[pi/4,5,10,pi/2],[pi/2,4,1,pi/2],[0,5,3,0]])
rho = numpy.array([1,1,1,1])
dq = numpy.array([1,1,1,1])
ddq = numpy.array([1,1,1,1])
joint_config = numpy.array([1,1,1,1])

#def newtonEuler()

# Compute angular and linear velocity for force component
def velAndAccel(dh_table, dq,ddq, joint_config):
	#compute rotational and linear velocity and acceleration
	n = len(dh_table)
	g = 9.81			#gravitational acceleration (scalar)
	
	w_i = numpy.array([[0],[0],[0]])		#ground angular velocity
	dw_i = numpy.array([[0],[0],[0]])		#ground angular acceleration
	v_i = numpy.array([[0],[0],[0]])		#initial linear velocity
	dv_i = numpy.array([[0],[0],[-g]])		#gravitational acceleration
	z = numpy.array([[0],[0],[1]])
	
	if len(joint_config) != len(dh_table):
		print "cannot find solution"
	
	else:
		for i in range (0,n):
			R = getRotMatrix(i,dh_table)
			P = getPosMatrix(i,dh_table)
			#print "P",i+1
			#print P, P.shape	
			
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
				#print w_i_T
				#print w_i_T.shape
			
				#angular acceleration
				dw_ip1 = numpy.dot(R,dw_i) + numpy.cross(numpy.dot(R,w_i).T,numpy.dot(dq[i],z).T).T + numpy.dot(ddq[i],z)
			
				dw_i = dw_ip1
			
				#linear acceleration
				comp1 = numpy.cross(dw_i.T,P)
				comp2 = comp1.T+numpy.cross(dw_i.T,comp1).T+dv_i
				dv_ip1 = numpy.dot(R,comp2)
				dv_i = dv_ip1
				
				#linear acceleration at center of mass
				#P_Ci = forwardPositionKinematics(i,dh_table,cm)
				#let component1 = dw_i x P_ci
				#comp_c1 = numpy.cross(dw_i.T,P_ci)
				#dv_ci = comp_c1.T + numpy.cross(w_i.T,comp_c1).T+dv_i
				
			else:
				print "not a standard joint configuration"
	
	#return w_i		
	return w_i, dw_i, dv_i
	
#algorithm test
w_1, dw_1, dv_1 = velAndAccel(dh_table,dq,ddq,joint_config)
print w_1
print w_1.shape
print dw_1
print dw_1.shape
print dv_1
print dv_1.shape
#w_1 = velAndAccel(dh_table,dq,ddq,joint_config)







