#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from cisst_msgs.msg import vctDoubleVec
from sensor_msgs.msg import JointState

def callback(data):
	pos = data.position
	rospy.loginfo(rospy.get_caller_id(),data.data)
	print(pos)

def pos_sub():
	rospy.init_node('pos_sub',anonymous=True)
	rospy.Subscriber("joint_states/joint_position_current",JointState, callback)

	rospy.spin()

def effort_sub():
	rospy.init_node('effort_sub',anonymous=True)
	rospy.Subscriber("joint_states/")

if __name__ == '__main__':
	pos_sub()

print pos_sub