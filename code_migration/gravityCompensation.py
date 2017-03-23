#!/usr/bin/env python
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from cisst_msgs.msg import vctDoubleVec

from newtonEulerMTM import *
from daVinci_param import *

#public torque for each jointeffort 

#since this algorithm is based on velocity and acceleration, we must limit those varables to 0
#in order to maintain the current configuration (compensate the gravity)

#steps: subscribe joint positions --> calculate torque --> publish torque
def pos_cb(data):
	pos = data.position
	rospy.loginfo(rospy.get_call_id())
	print(pos)

def sub_pos():
	rospy.init_node('sub_pos',anomymous=True)
	#the topic's name must be changed accordingly
	rospy.Subscriber("/dvrk/MTML/joint_states",JointStates,pos_cb)

	rospy.spin()

q = numpy.array([pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6]])

dh_table = davinci_param(q)
joint_config = numpy.array([1,1,1,1,1,1,1])
dq = numpy.array([0,0,0,0,0,0,0])
ddq = numpy.array([0,0,0,0,0,0,0])

torque = newtonEulerMTM(dh_table, dq,ddq, joint_config)

def gravityCompensation():
	msg = JointState()
	#all joints are revolute joint
	
	rospy.init_node('gravityCompensation', anonymous=true)
	#publish torque (which currently i have no idea what the topic is called)
	pub_tor = rospy.Publisher('/dvrk_mtm/set_joint_effort',Float64MultiArray,queue_size=10)



