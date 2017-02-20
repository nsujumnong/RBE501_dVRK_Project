#!/usr/bin/env python

import numpy

from numpy import *

def linearAccel(R,dw_i,P,dv_i,d_d,dd_d,joint_config):
	if joint_config == 0: 
		dw_P_i = numpy.cross(dw_i,P)
		w_P_i = numpy.cross(w_i,P)
		dv_ip1 = numpy.dot(R,dw_P_i + numpy.cross(w_P_i) + dv_i) + numpy.dot(2.0,w_ip1)

	elif joint_config == 1:
		dw_P_i = numpy.cross(dw_i.T,P).T
		dv_ip1 = numpy.dot(R,dw_P_i + numpy.cross(dw_i.T,dw_P_i.T).T + dv_i)
		dv_i = dv_ip1
	
	return dv_i
	
