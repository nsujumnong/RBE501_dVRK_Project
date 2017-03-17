#!/usr/bin/env python
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from cisst_msgs.msg import vctDoubleVec

from newtonEulerMTM import *

#public torque for each jointeffort 

#since this algorithm is based on velocity and acceleration, we must limit those varables to 0
#in order to maintain the current configuration (compensate the gravity)

#desired velocity and acceleration
desired_vel = numpy.array([0,0,0,0,0,0,0])
desired_acc = numpy.array([0,0,0,0,0,0,0])
L = numpy.array([0.195, 0.285, 0.370, 0.115, 0.150, 11.5, 0.0725, 0.0725, 0.060])
theta = numpy.array([[q[0]], [-q[1]+pi/2.0], [-q[2]-pi/2.0], [q[3]], [-q[4]-pi], [q[5]-pi/2.0], [-q[6]]])
d = numpy.array([[-1.0*L[0]], [0], [0], [L[4]], [0], [0], [0]])
a = numpy.array([[0], [L[1]], [L[2]], [0], [0], [0], [0]])
alpha = numpy.array([[-pi/2.0], [0], [pi/2.0], [-pi/2.0], [-pi/2.0], [pi/2.0], [0]])
dh_table = numpy.concatenate((theta,d,a,alpha),axis=1)
joint_config = numpy.array([1,1,1,1,1,1,1])

#steps: subscribe joint positions --> calculate torque --> publish torque
def pos_cb(data):
	pos = data.position
	rospy.loginfo(rospy.get_call_id())
	print(pos)

def sub_pos():
	rospy.init_node('sub_pos',anomymous=True)
	rospy.Subscriber("/dvrk/MTML/joint_states",JointStates,pos_cb)

	rospy.spin()

torque = newtonEulerMTM()

def gravityCompensation():
	msg = JointState()
	#all joints are revolute joint
	
	rospy.init_node('gravityCompensation', anonymous=true)
	#publish torque (which currently i have no idea what the topic is called)
	pub_tor = rospy.Publisher('/dvrk_mtm/set_joint_effort',Float64MultiArray,queue_size=10)



