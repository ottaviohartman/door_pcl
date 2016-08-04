#!/usr/bin/env python

import serial, time, struct, rospy, math, tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu
from rositaur.msg import Command
from sensor_msgs.msg import Joy

pub = rospy.Publisher('/command', Command, queue_size=100)
state = 0

cmd_kill = 0
cmd_stand = 1
cmd_bound = 2

def doorCallback(data):
	rospy.loginfo("Door is at point: %f, %f. Height: %f", data.point.x, data.point.y, data.point.z)
	
	rot = math.atan2(data.point.y, data.point.x)
	
	rotDes = rot / 2.

	rospy.loginfo("Turning. rot: %f", rotDes)
	
	msg = Command()
	msg.command = state
	
	msg.param1 = 0.0
	msg.param2 = 0.0

	#msg.param1 = data.axes[3] * .3 
	#msg.param2 = rotDes
	
	rospy.loginfo(msg)
	pub.publish(msg)

def joyCallback(data):
	global state
	rospy.loginfo("Changing state to %i", state)

	#X
	if (data.buttons[2] == 1):
		state = cmd_kill
	#A
	elif (data.buttons[0] == 1):
		state = cmd_stand
	#B
	elif (data.buttons[1] == 1):
		state = cmd_bound
		
def initListener():
	rospy.init_node('robot_control', anonymous=True)

	rospy.Subscriber("/door", PointStamped, doorCallback)	
	rospy.Subscriber("/joy", Joy, joyCallback)
	

if __name__ == '__main__':
	initListener()
	rospy.spin()

