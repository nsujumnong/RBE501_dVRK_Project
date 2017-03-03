#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from cisst_msgs.msg import vctDoubleVec

def callback(data):
	tup = data.data
    rospy.loginfo(rospy.get_caller_id())
    print (tup)
    str = type(tup)
    print(str)
#    list(tup)
#    str = type(tup)
#    print(str)
    msg.layout.dim = []
    msg.layout.data_offset = 0
    msg.data = tup
    print(msg.data)
    pub.publish(msg)



def tor_listener():

	rospy.init_node('tor_listener', anonymous=True)

	rospy.Subscriber('/dvrk/MTMR/get_current_joint_effort',vctDoubleVec, callback)

	rospy.spin()

if __name__ == '__main__':
	tor_listener()