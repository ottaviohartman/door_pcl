#!/usr/bin/env python

import serial, time, struct, rospy, math, tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu

import message_filters

#ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)

def callback(rotData, doorData):
	print("Calling the callback")
	rospy.loginfo("Got quaternion: %f, %f, %f, %f", rotData.orientation.x, rotData.orientation.y, rotData.orientation.z, rotData.orientation.w)
	(_, _, tfRot) = tf.transformations.euler_from_quaternion([rotData.orientation.x, rotData.orientation.y, rotData.orientation.z, rotData.orientation.w])
	
	#rospy.loginfo("tf rot: %f", tfRot)
	rospy.loginfo("Door is at point: %f, %f. Height: %f", doorData.point.x, doorData.point.y, doorData.point.z)
	rot = math.atan2(doorData.point.y, doorData.point.x)

	rotDes = tfRot - rot

	if (math.fabs(rotDes) > .2):
		rotDes /= 2.
		#sendCmd(1, 0.0, rotDes)
		rospy.loginfo("Turning. tfRot: %f rot: %f rotDes: %f", tfRot, rot, rotDes)
	else:
		speed = .1
		#sendCmd(1, speed, 0.0)
		#rospy.loginfo("Bounding: %f", speed)

def initListener():
	rospy.init_node('robot_control', anonymous=True)

	door_sub = message_filters.Subscriber('/door', PointStamped)
	rot_sub = message_filters.Subscriber('/imu/data', Imu)

	ts = message_filters.ApproximateTimeSynchronizer([rot_sub, door_sub], 10, 1)
	ts.registerCallback(callback)

	time.sleep(3)

	print 'Stand'
	#sendCmd(0, 0.0, 0.0)
	time.sleep(5)

	print 'Start bound'
	#sendCmd(1, 0.0, 0.0)
	time.sleep(2)

	rospy.spin()


def sendCmd(bound, speedDes, yawDes):
  cmd = bytearray(struct.pack('<BBff',123,bound,speedDes,yawDes))
  ser.write(cmd)

if __name__ == '__main__':

	initListener()

	# time.sleep(3)
	# print 'Stand'
	# sendCmd(0, 0.0, 0.0)

	# time.sleep(5)
	# print 'Start bound'
	# sendCmd(1, -0.1, 0.0)

	# time.sleep(2)
	# print 'Set speed 0.2'
	# sendCmd(1, 0.1, 0.0)

	# time.sleep(2)
	# print 'Stand'
	# sendCmd(0, 0.0, 0.0)

	# ser.close()