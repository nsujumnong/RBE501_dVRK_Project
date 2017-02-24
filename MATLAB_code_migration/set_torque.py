#!/usr/bin/env python
import rospy

<<<<<<< HEAD

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from cisst_msgs.msg import vctDoubleVec

from newtonEulerMTM import *

#get position node from daVinci
#public torque for each jointeffort 
#compute the position error

#subscribe for desired position here
def gravityCompensation():
	msg = JointState()
	#all joints are revolute joint
	joint_config = numpy.array([1,1,1,1,1,1,1])
	rospy.init_node('gravityCompensation', anonymous=true)
	sub_pos = rospy.Subscriber('/dvrk_mtm/joint_position_current',Float64MultiArray)
	pub_tor = rospy.Publisher('/dvrk_mtm/set_joint_effort',Float64MultiArray,queue_size=10)

	#initiate position error
	position_error = numpy.zeros((8,100))

	for i in range (0,99):
		#get current position by subscribing to '/dvrk_mtm/joint_position_current' here
		torque = newtonEuler(dh_table,dq,ddq,joint_config)
		#implement torque by publishing to '/dvrk_mtm/set_joint_effort' here
		#set_torque(position,torque)
		#need delay factor too
		rospy.sleep(0.01)
		position_error[0:7,i] = #current position - desired position


def set_torque(position,torque):

	msg.Effort[0] = torque[0]
	msg.Effort[1] = torque[1]
	msg.Effort[2] = torque[2]
	msg.Effort[3] = torque[3]
	msg.Effort[4] = torque[4]
	msg.Effort[5] = torque[5]
	msg.Effort[6] = torque[6]


=======
def set_torque():
	
>>>>>>> parent of b1b8104... edited 'set_torque' and add some description
