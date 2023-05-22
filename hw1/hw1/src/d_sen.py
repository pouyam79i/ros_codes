#!/usr/bin/python3

import rospy
from hw1.msg import distance
import random

def generate_distance():
    north = random.randint(10,200)
    south = random.randint(10,200)
    west  = random.randint(10,200)
    east  = random.randint(10,200)
    return north, south, west, east

def talker():
    rospy.init_node("sensor", anonymous=True)
    pub = rospy.Publisher("distance", distance, queue_size=10)
    rate = rospy.Rate(hz=1)

    while not rospy.is_shutdown():
        msg = distance()
        msg.north, msg.south, msg.west, msg.east = generate_distance()
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    talker()
