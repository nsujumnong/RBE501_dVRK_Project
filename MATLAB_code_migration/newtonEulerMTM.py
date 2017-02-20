#!/usr/bin/env python
import rospy

from numpy import *
from dh_transform import dhTransform
from forwardKin import *
from getPosMatrix import getPosMatrix
from getRotMatrix import getRotMatrix
from angularVel import *
from angularAccel import *
from linearAccel import *

pi = numpy.pi
#q1,q2,q3,q4,q5,q6,q7 = symbols('q1 q2 q3 q4 q5 q6 q7',real=True)
q = numpy.array([1.0,1.0,1.0,1.0,1.0,1.0,1.0])
## This is an exact copy of newtonEuler.py with real parameters from daVinci MTM ## 
## some parameters must be acquired through the actual robot, so right now some of them will be assumed as constant 1
# Link length
#link length = numpy.array([L1 L2 L3 L4_y0 L4_z0 L5_y0 L5_z0 L6_z0 L6_x0])
L = numpy.array([0.195, 0.285, 0.370, 0.115, 0.150, 11.5, 0.0725, 0.0725, 0.060])
theta = numpy.array([[q[0]], [-q[1]+pi/2.0], [-q[2]-pi/2.0], [q[3]], [-q[4]-pi], [q[5]-pi/2.0], [-q[6]]])
d = numpy.array([[-1.0*L[0]], [0], [0], [L[4]], [0], [0], [0]])
a = numpy.array([[0], [L[1]], [L[2]], [0], [0], [0], [0]])
alpha = numpy.array([[-pi/2.0], [0], [pi/2.0], [-pi/2.0], [-pi/2.0], [pi/2.0], [0]])
dh_table = numpy.concatenate((theta,d,a,alpha),axis=1)

#print dh_table
dq = numpy.array([1,1,1,1,1,1,1])
ddq = numpy.array([1,1,1,1,1,1,1])
d_d = numpy.array([1,1,1,1,1,1,1])
dd_d = numpy.array([1,1,1,1,1,1,1])
joint_config = numpy.array([1,1,1,1,1,1,1])

cm1 = numpy.array([[0],[-L[0]/2.0],[0]])
cm2 = numpy.array([[-L[1]/2],[0],[0]])
cm3 = numpy.array([[-L[2]/2],[0],[0]])
cm4 = numpy.array([[0],[3.0/4.0*L[4]],[3.0/4.0*L[3]]])
cm5 = numpy.array([[0],[-3.0/4.0*L[5]],[-3.0/4.0*L[6]]])
cm6 = numpy.array([[0],[-3.0/4.0*L[7]],[-3.0/4.0*L[8]]])
cm7 = numpy.array([[0],[0],[0]])

cm = numpy.concatenate((cm1,cm2,cm3,cm4,cm5,cm6,cm7),axis=1)
#print cm
#still assume all m to be 1
m = numpy.array([0.8,0.10,0.10,0.05,0.05,0.05,0])
#print cm

# The Newton-Euler algorithm computes the torque for each joint of the manipulator
def newtonEuler(dh_table, dq,ddq, joint_config):
	#compute rotational and linear velocity and acceleration
	n = len(dh_table)
	g = 9.81			#gravitational acceleration (scalar)
	
	w_i = numpy.array([[0],[0],[0]])		#ground angular velocity
	dw_i = numpy.array([[0],[0],[0]])		#ground angular acceleration
	v_i = numpy.array([[0],[0],[0]])		#initial linear velocity
	dv_i = numpy.array([[0],[0],[-g]])		#gravitational acceleration
	z = numpy.array([[0],[0],[1]])			#unit vector of z-axis
	f_ip1 = numpy.array([[0],[0],[0]])		#initial force at the end of the manipulator (assume that it is not interacting with environment)
	
	if len(joint_config) != len(dh_table):
		print "cannot find solution"
	
	else:
		for i in range (0,n):
			R = getRotMatrix(i,dh_table)
			P_c = getPosMatrix(i,dh_table,cm)	
			#forward iteration
			w_i = angularVel(R,w_i,dq[i],z,joint_config[i])
			dw_i = angularAccel(R,w_i,dw_i,dq[i],ddq[i],z,joint_config[i])
			dv_i = linearAccel(R,dw_i,P,dv_i,joint_config[i])
			print "w_",i
			print w_i
				
			##linear acceleration at center of mass
			#compute position of the center of mass of link ith
			P_Ci = forwardPositionKinematics(i,dh_table,cm)
			P_Ci = P_Ci.reshape((3,1))
			#let component1 = dw_i x P_ci
			comp_c1 = numpy.cross(dw_i.T,P_Ci.T)
			dv_ci = comp_c1.T + numpy.cross(w_i.T,comp_c1).T+dv_i
				
			#compute force and moment components
			F_i, N_i = forceMoment(m[i],dv_i,I_ci,w_i,dw_i)
			
			#backward iteration (force and moment)
			f_i = numpy.dot(R,f_ip1)+F_i
			f_ip1 = f_i
			n_i = N_i + numpy.dot(R,n_ip1) + numpy.cross(P_Ci,F_i) + numpy.cross(P_i,numpy.dot(R,f_ip1))
			n_ip1 = n_i
					
	return w_i, dw_i, dv_i, P_Ci, dv_ci
	
#algorithm test
w_i, dw_i, dv_i, P_Ci, dv_ci = newtonEuler(dh_table,dq,ddq,joint_config)
print "w_i"
print w_i
print "dw_i"
print dw_i
print "dv_i"
print dv_i
print "P_Ci"
print P_Ci
print "dv_ci"
print dv_ci

