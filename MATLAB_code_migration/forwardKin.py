#!/usr/bin/env python
import rospy
import numpy
import dh_transform

from numpy import *

pi = numpy.pi

dh_table = numpy.array([[pi/6,2,3,0],[pi/4,5,10,pi/2],[pi/2,4,1,pi/2],[0,5,3,0]])
dh_table1 = dh_table[0]
