#!/usr/bin/env python
import numpy

from numpy import *

q = numpy.array([0,0,0,0,0,0,0])

def davinci_param(q):
	L = numpy.array([0.195, 0.285, 0.370, 0.115, 0.150, 11.5, 0.0725, 0.0725, 0.060])
	theta = numpy.array([[q[0]], [-q[1]+pi/2.0], [-q[2]-pi/2.0], [q[3]], [-q[4]-pi], [q[5]-pi/2.0], [-q[6]]])
	d = numpy.array([[-1.0*L[0]], [0], [0], [L[4]], [0], [0], [0]])
	a = numpy.array([[0], [L[1]], [L[2]], [0], [0], [0], [0]])
	alpha = numpy.array([[-pi/2.0], [0], [pi/2.0], [-pi/2.0], [-pi/2.0], [pi/2.0], [0]])
	dh_table = numpy.concatenate((theta,d,a,alpha),axis=1)

	return dh_table

# table = davinci_param(q)
# print(table)