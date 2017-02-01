#!/usr/bin/env python
import rospy
import numpy

from numpy import *

pi = numpy.pi


def dhTransform(theta,d,a,alpha):
	T = numpy.dot(numpy.dot(rotationMatrix(theta,'z'),translationMatrix(d,'z')),numpy.dot(translationMatrix(a,'x'),rotationMatrix(alpha,'x')))

	return T


def translationMatrix(d,t_ax):
	
	if t_ax == 'x':
		T = numpy.array([[0,0,0,d],[0,0,0,0],[0,0,0,0],[0,0,0,1]])

	elif t_ax == 'y':
		T = numpy.array([[0,0,0,0],[0,0,0,d],[0,0,0,0],[0,0,0,1]])

	elif t_ax == 'z':
		T = numpy.array([[0,0,0,0],[0,0,0,0],[0,0,0,d],[0,0,0,1]])

	elif t_ax == 'all':
		T = numpy.array([[0,0,0,d],[0,0,0,d],[0,0,0,d],[0,0,0,1]])	

	else:
		print('invalid axis')

	return	T


def rotationMatrix(theta,r_ax):
	
	if r_ax == 'x':
		R = numpy.array([[1,0,0,0],[0,math.cos(theta),-math.sin(theta),0],[0,math.sin(theta),math.cos(theta),0],[0,0,0,1]])
	
	elif r_ax == 'y':
		R = numpy.array([[math.cos(theta),0,math.sin(theta),0],[0,1,0,0],[-math.sin(theta),0,math.cos(theta),0],[0,0,0,1]])
	
	elif r_ax == 'z':
		R = numpy.array([[math.cos(theta),-math.sin(theta),0,0],[math.sin(theta),math.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
	else:
		print('invalid axis')
		
	return R

rot1 = rotationMatrix(pi/6,'z')
print rot1
transl1 = translationMatrix(10,'z')
print transl1
transl2 = translationMatrix(20,'x')
print transl2
rot2 = rotationMatrix(pi/2,'x')
print rot2
z_ax = numpy.dot(rot1,transl1)
x_ax = numpy.dot(transl2,rot2)
print z_ax
print x_ax

#dh_trans = dhTransform(pi/2,24,10,pi/3)
#print dh_trans
