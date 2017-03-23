#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState

#testing code for joint state subscription

def callback(data):
	tup = data.position
	rospy.loginfo(rospy.get_caller_id())
	print(tup)
	print(tup[0])

def pos_sub():
	rospy.init_node('pos_sub',anonymous=True)
	rospy.Subscriber("/dvrk/MTMR/joint_states",JointState, callback)

	rospy.spin()

if __name__ == '__main__':
	pos_sub()
