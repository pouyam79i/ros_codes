#!/usr/bin/python3

import rospy
import tf
from laser_scan.msg import closest_obstacle
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import radians, atan2, sqrt, inf, pi

class Pose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

class Controller:
    def __init__(self, linear_speed, angular_speed) -> None:
      
        # Init node
        self.node = rospy.init_node('control_node', anonymous=False)

        # Command Publisiher
        self.cmd_publisher =  rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribing sensor
        self.laser_scan = rospy.Subscriber('/obstacle_detector', closest_obstacle ,callback=self.get_closest_obstacle)

        # Speeds
        self.linear_speed = linear_speed     # m/s
        self.angular_speed = angular_speed   # rad/s

        # state of robot
        self.ROTATE, self.GO, self.STOP = 0, 1, -1
        self.state = self.STOP

        # last obstacle info
        self.co_direction = -1
        self.co_distance = -1
        
        # pose data
        self.pose = Pose()
        refresh = 10
        while refresh > 0:
            self.update_pose() #first update
            refresh -= 1

        # epsilone error
        self.epsilone = 0.01

        # error sum
        self.e_num = 0.0
        self.e_sum = 0.0

    # update current pose
    def update_pose(self):
        msg = rospy.wait_for_message('odom', Odometry)
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion((
            orientation.x, orientation.y, orientation.z, orientation.w
        ))
        position = msg.pose.pose.position
        self.pose.x = position.x
        self.pose.y = position.y
        self.pose.yaw = yaw + pi # calculate from 0 to 2pi

    # just returns pose
    def get_pose(self):
        obj = Pose()
        obj.x, obj.y, obj.yaw = self.pose.x, self.pose.y, self.pose.yaw
        return obj

    # get next pose from mission server
    def get_closest_obstacle(self, msg: closest_obstacle):
        self.co_direction = msg.direction 
        self.co_distance = msg.distance
        return self.co_direction, self.co_distance

    # send cmd
    def send_twist(self, linear_speed, angular_speed):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_publisher.publish(twist)

    # update error
    def update_error(self, error):
        self.e_sum += error
        self.e_num += 1

    # there must be a next pose to go
    def change_direction(self, co_direction, co_distance):

        # No need to change direction
        if co_direction < 0 or co_distance < 0:
            return 

        # No need to change for obstacle in opposite direction
        if co_direction <= 265 and co_direction >= 95:
            return

        # Stop
        self.send_twist(0, 0)
        self.state = self.STOP
        rospy.sleep(rospy.Duration(3))
  
        # Rotate
        self.state = self.ROTATE
        self.update_pose()
        pose = self.get_pose()
        obstacle_angel = radians(co_direction) % (2*pi)
        rotation_angel = (obstacle_angel + pi) % (2*pi)
        final_yaw = (pose.yaw + rotation_angel) % (2*pi)
        while True:
            self.update_pose()
            pose = self.get_pose()
            diff = final_yaw - pose.yaw
            theta = pi
            rotate_right = False
            if diff < 0:
                if diff < (-pi):
                    theta = 2*pi + diff
                    rotate_right = False
                else:
                    theta = abs(diff)
                    rotate_right = True
            else:
                if diff > pi:
                    theta = 2 * pi - diff
                else:
                    rotate_right = False
                    theta = diff
            if theta < self.epsilone:
                self.send_twist(0, 0)
                self.state = self.STOP
                rospy.sleep(rospy.Duration(3))
                break
            if rotate_right:
                self.send_twist(0, self.angular_speed * -1)
            else:
                self.send_twist(0, self.angular_speed)
            rospy.sleep(rospy.Duration(0.1))

    # run controller node
    def Run(self):
        while not rospy.is_shutdown():
            self.change_direction(self.co_direction, self.co_distance)
            self.state = self.GO
            self.send_twist(self.linear_speed, 0)
            rospy.sleep(rospy.Duration(0.1))


# Run code
if __name__ == "__main__":
    # linear_speed = rospy.get_param('linear_speed')
    # angular_speed = rospy.get_param('angular_speed')

    linear_speed = 0.8
    angular_speed = 0.1

    ctl = Controller(linear_speed, angular_speed)
    ctl.Run()
