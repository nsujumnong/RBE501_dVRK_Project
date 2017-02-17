#!/usr/bin/env python

import numpy

from numpy import *

def forceMoment(m,dv_i,I_ci,dw_i):
	#Force component
	F_i = numpy.dot(m[i],dv_ci)
	#Normal force
	I_dw_dot = numpy.dot(I_ci,dw_i)
	N_i = I_dw_dot + numpy.cross(w_i,I_dw_dot)

	return F_i, N_i

