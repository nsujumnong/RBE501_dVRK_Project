#!/usr/bin/env python

import numpy

from numpy import *

def linearAccel(R,dw_i,P,dv_i,joint_config):
	if joint_config == 0: 
		print "lol ain't gonna do that... for now"
	elif joint_config == 1:
		dw_P_i = numpy.cross(dw_i.T,P).T
		dv_ip1 = numpy.dot(R,dw_P_i + numpy.cross(dw_i.T,dw_P_i.T).T+dv_i)
		dv_i = dv_ip1
	
	return dv_i
	
