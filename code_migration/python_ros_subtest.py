#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState

def callback(data):
	tup = data
	rospy.loginfo(rospy.get_caller_id())
	print(tup)

def pos_sub():
	rospy.init_node('pos_sub',anonymous=True)
	rospy.Subscriber("/dvrk/MTML/joint_states",JointState, callback)

	rospy.spin()

if __name__ == '__main__':
	pos_sub()
