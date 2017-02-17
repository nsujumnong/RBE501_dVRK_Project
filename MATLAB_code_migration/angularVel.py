#!/usr/bin/env python

import numpy

from numpy import *

#compute the angular velocity of each link

def angularVel(R,w_i,dq,z,joint_config):
	if joint_config == 0:
		w_ip1 = numpy.dot(R,w_i)
		w_i = w_ip1
	elif joint_config == 1:
		w_ip1 = numpy.dot(R,w_i)+numpy.dot(dq,z)
		w_i = w_ip1
	return w_i
