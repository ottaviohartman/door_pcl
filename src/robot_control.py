#!/usr/bin/env python

import serial, time, struct, rospy, math, tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu

tfRot = 0
#ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)

def doorCallback(data):
	rospy.loginfo("Door is at point: %f, %f. Height: %f", data.point.x, data.point.y, data.point.z)
	rot = math.atan2(data.point.y, data.point.x)

	rotDes = rot - tfRot

	if (math.fabs(rotDes) > .2):
		sendCmd(1, 0.0, rotDes)
		rospy.loginfo("Turning: %f", rotDes)
	else:
		speed = .1
		sendCmd(1, speed, 0.0)
		rospy.loginfo("Bounding: %f", speed)

def rotCallback(data):
	# sets the global variable tfRot
    tfRot = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])[2]
    #rospy.loginfo("tf rot: %f", tfRot)

def initListener():
	rospy.init_node('robot_control', anonymous=True)

	rospy.Subscriber("door", PointStamped, doorCallback)	
	rospy.Subscriber("/imu/data", Imu, rotCallback)

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