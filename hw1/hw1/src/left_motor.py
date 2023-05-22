#!/usr/bin/python3

import rospy
from hw1.msg import motor

def callback(data):
    rospy.loginfo("\ndegree: %d, direction:%s", data.degree, data.direction)

def motor_left():
    rospy.init_node('M_left', anonymous=True)
    rospy.Subscriber('motor_left', motor, callback)
    rospy.spin()

if __name__ == '__main__':
    motor_left()
