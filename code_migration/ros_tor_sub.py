#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from cisst_msgs.msg import vctDoubleVec

def callback(data):
	



def tor_listener():

	rospy.init_node('tor_listener', anonymous=True)

	rospy.Subscriber('/dvrk/MTMR/got_current_joint_effort',vctDoubleVec, callback)

	rospy.spin()

if __name__ == '__main__':
	tor_listener()