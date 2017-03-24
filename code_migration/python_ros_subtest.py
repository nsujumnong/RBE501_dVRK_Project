#!/usr/bin/env python

import rospy
import numpy

from numpy import *

from sensor_msgs.msg import JointState

#testing code for joint state subscription

def callback(data):
	tup = data.position
	rospy.loginfo(rospy.get_caller_id())
	
	# print(tup)
	# print(tup[0])

def pos_sub():
	pos = rospy.init_node('pos_sub',anonymous=True)
	rospy.Subscriber("/dvrk/MTMR/joint_states",JointState, callback)

	rospy.spin()

if __name__ == '__main__':
	pos_sub()

# q = numpy.array([tup[0],tup[1],tup[2],tup[3],tup[4],tup[5],tup[6]])
q = callback()