#!/usr/bin/python3

import rospy
from hw1.msg import distance
from hw1.msg import motor

# publisher of controller
global pub_ml
global pub_mr
# Robot_head degree 
# Degree of 0 means north, so the rest is (w:90, s:180, e:270)
global robot_head

# here we produce rotating degree
# true means right, false means left
def logic(data):
    closest_degree = 0
    min_dis = min(data.north, data.west, data.south, data.east)
    if min_dis == data.north:
        closest_degree = 0
    elif min_dis == data.west:
        closest_degree = 90
    elif min_dis == data.south:
        closest_degree = 180
    elif min_dis == data.east:
        closest_degree = 270

    if closest_degree <= 180:
        return closest_degree, False
    return 360 - closest_degree, True

def decision(data):
    global robot_head
    degree, lr = logic(data)
    
    motor_right = 'stop'
    motor_left  = 'stop'

    if degree == 0:
        return 0, motor_left, motor_right
    elif lr:
        motor_left  = 'forward'
        motor_right = 'backward'
        robot_head = robot_head - degree
    else:
        motor_left  = 'backward'
        motor_right = 'forward'
        robot_head = robot_head + degree

    if robot_head < 0:
        robot_head = robot_head + 360
    elif robot_head >= 360:
        robot_head = robot_head - 360

    return degree, motor_left, motor_right

def callback(data):
    d, ml, mr = decision(data)
    motor_left  = motor()
    motor_right = motor()
    motor_left.degree = d
    motor_right.degree = d
    motor_left.direction = ml
    motor_right.direction = mr
    pub_ml.publish(motor_left)
    pub_mr.publish(motor_right)
    rospy.loginfo("\nDecision => d:%d, ml:%s, mr:%s - RobotHead: %d", d, ml, mr, robot_head)

def controller():
    global pub_ml
    global pub_mr
    global robot_head
    robot_head = 0
    pub_ml = rospy.Publisher('motor_left', motor, queue_size=1)
    pub_mr = rospy.Publisher('motor_right', motor, queue_size=1)
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber('distance', distance, callback)
    rospy.spin()


if __name__ == '__main__':
    controller()
