#!/usr/bin/env python

import serial, time, struct, rospy, math, tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu

tfRot = 0
ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)

def doorCallback(data):
	rospy.loginfo("Door is at point: %f, %f. Height: %f", data.point.x, data.point.y, data.point.z)
	rot = math.atan2(data.point.y, data.point.x)

	rotDes = tfRot - rot

	rotDes /= 3.
	#if (math.fabs(rotDes) > .2):
	sendCmd(1, 0.0, rotDes)
	rospy.loginfo("Turning. tfRot: %f rot: %f rotDes: %f", tfRot, rot, rotDes)
	#else:
	#	speed = .1 * (data.point.x**2 + data.point.y**2)
	#	if speed > .2:
	#		speed = .2
	#	sendCmd(1, speed, 0.0)
	#	rospy.loginfo("Bounding: %f", speed)

def rotCallback(data):
	#rospy.loginfo("Calling rotCallback")
	# sets the global variable tfRot
	global tfRot
	#rospy.loginfo("Got quaternion: %f, %f, %f, %f", data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
	(_, _, tfRot) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
	
	tfRot *= -1
	#tfRot = math.pi + tfRot
	#rospy.loginfo("tf rot: %f", tfRot)

def initListener():
	rospy.init_node('robot_control', anonymous=True)

	rospy.Subscriber("/imu/data", Imu, rotCallback)
	rospy.Subscriber("/door", PointStamped, doorCallback)	

	rospy.sleep(3)
	rospy.loginfo('Stand')
	sendCmd(0, 0.0, 0.0)
	rospy.sleep(5)
	rospy.loginfo('Start bound')
	sendCmd(1, -0.1, 0.0)
	rospy.sleep(2)
	sendCmd(1, 0.0, 0.0)

	rospy.spin()


def sendCmd(bound, speedDes, yawDes):
  cmd = bytearray(struct.pack('<BBff',123,bound,speedDes,yawDes))
  ser.write(cmd)

if __name__ == '__main__':

	initListener()
