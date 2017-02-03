#!/usr/bin/env python
import rospy
import numpy

from dh_transform import dhTransform
from dh_transform import translationMatrix
from numpy import *

pi = numpy.pi

dh_table = numpy.array([[pi/6,2,3,0],[pi/4,5,10,pi/2],[pi/2,4,1,pi/2],[0,5,3,0]])


def forwardKinematics(i,dh_table):

	T = numpy.identity(4)
	print i
	for j in range (0,i):
		print "this is loop: ",j+1
		ith_dh_table = dh_table[j]
		theta = ith_dh_table[0]
		d = ith_dh_table[1]
		a = ith_dh_table[2]
		alpha = ith_dh_table[3]
		T_trans = dhTransform(theta,d,a,alpha)
		T = numpy.dot(T,T_trans)
	cm_transl = translationMatrix(cm[:,i-1],'all')
	ith_T_cm = numpy.dot(T,cm_transl)	
	return ith_T_cm

table_ith = forwardKinematics(i,dh_table)
print table_ith

