#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
from laser_scan.msg import closest_obstacle
from math import inf

# Gets information from sensor
class Sensor:

    def __init__(self) -> None:
        self.laser_scanner = rospy.Subscriber('/scan', LaserScan ,callback=self.sensor_reader)
        self.ranges = [inf]
        self.limit = 2  # 2m limitation

    # update sensor data
    def sensor_reader(self, msg: LaserScan):
        self.ranges = msg.ranges

    # get ranges value
    def get_ranges(self):
        return self.ranges

    # returns closest object direction and distance (order is important)
    def get_closest_obstacle(self):
        ranges = self.get_ranges()
        min_direction = -1
        min_distance = inf
        for i in range(len(ranges)):
            if ranges[i] <= self.limit and ranges[i] < min_distance: 
                min_direction = i
                min_distance = ranges[i]        
        if min_direction == -1:
            min_distance = -1
        return min_direction, min_distance # obstacles are far

# node worker
def run():
   rospy.init_node('sensor_node', anonymous=True)
   pub = rospy.Publisher('obstacle_detector', closest_obstacle, queue_size=10)
   sensor = Sensor()
   while not rospy.is_shutdown():
    msg = closest_obstacle()
    msg.direction, msg.distance = sensor.get_closest_obstacle()
    pub.publish(msg)
    rospy.sleep(rospy.Duration(0.1))

if __name__ == "__main__":
    run()
