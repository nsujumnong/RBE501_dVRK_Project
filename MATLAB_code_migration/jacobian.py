#!/usr/bin/env python

import numpy as np
import sympy
import scipy

from numpy import *
from sympy import *

pi = math.pi

#def jacobian(f,v):
#	n = len(f)
#	m = len(v)
#	for i in range (0,n):
#		for j in range
	
	

#L = range(10)
#print L
#R = L[:2]
#print R
	

#x = np.array([[1,2,3],[4,5,6],[7,8,9]])
	
x,y,z = symbols('x y z', real=True)
a,b,c = symbols('a b c', real=True)
f = 4*x*y + x*sin(z) + x**3 + z**8*y

#print diff(f,x)

sin_pi = sin(pi/2*x)
d_sin = diff(sin_pi,x)
print d_sin
