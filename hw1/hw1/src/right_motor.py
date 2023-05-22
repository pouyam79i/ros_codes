#!/usr/bin/python3

import rospy
from hw1.msg import motor

def callback(data):
    rospy.loginfo("\ndegree: %d, direction:%s", data.degree, data.direction)

def motor_right():
    rospy.init_node('M_right', anonymous=True)
    rospy.Subscriber('motor_right', motor, callback)
    rospy.spin()

if __name__ == '__main__':
    motor_right()
