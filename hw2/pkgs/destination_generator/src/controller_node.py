#!/usr/bin/python3

import rospy
import tf
from destination_generator.srv import GetNextDestination
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
        self.node = rospy.init_node('controller_node', anonymous=False)

        # Command Publisiher
        self.cmd_publisher =  rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # TODO: check befor fix subscriber Odometry
        # self.odometry = rospy.Subscriber('/odom', Odometry, callback=self.update_pose)

        # Speeds
        self.linear_speed = linear_speed     # m/s
        self.angular_speed = angular_speed   # rad/s
        self.goal_angel = radians(90)        # rad

        # state of robot
        self.ROTATE, self.GO, self.STOP = 0, 1, -1
        self.state = self.STOP

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
        self.pose.yaw = yaw

    # just returns pose
    def get_pose(self):
        obj = Pose()
        obj.x, obj.y, obj.yaw = self.pose.x, self.pose.y, self.pose.yaw
        return obj

    # get next pose from mission server
    def get_next_pose(self, x, y):
        rospy.wait_for_service('get_next_destination')
        try:
            srv_init = rospy.ServiceProxy('get_next_destination', GetNextDestination)
            res = srv_init(x, y)
            next_pose = Pose()
            next_pose.x = res.next_x
            next_pose.y = res.next_y
            return next_pose
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            rospy.sleep(rospy.Duration(1))
            return None

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
    def move(self, next_pose):

        # Null input 
        if next_pose == None:
            self.state = self.STOP
            return False
        
        # report
        self.update_pose()
        current_pose = self.get_pose()
        print("Moving from ({}, {}) to ({}, {})...".format(current_pose.x, current_pose.y, next_pose.x, next_pose.y))

        # do a rotation if needed
        self.state = self.ROTATE
        print("Robot is rotating...")
        current_pose = self.get_pose()
        dx = next_pose.x - current_pose.x
        dy = next_pose.y - current_pose.y
        theta = atan2(dy, dx)
        while True:
            self.update_pose()
            current_pose = self.get_pose()
            beta = theta - current_pose.yaw
            # print("#DEBUG - rotating pose ({}, {}, {}) - theta: {} - beta: {}".format(current_pose.x, current_pose.y, current_pose.yaw, theta, beta))
            # if difference is less then epsilone get out
            if abs(beta) % (2*pi) < self.epsilone:
                self.send_twist(0, 0)
                rospy.sleep(rospy.Duration(3))
                break
            beta = beta % (2*pi)
            if beta  > pi:
                self.send_twist(0, self.angular_speed * -1)
            else:
                self.send_twist(0, self.angular_speed)
            rospy.sleep(rospy.Duration(0.05))
        print("Rotation is Done!")
        print("Robot is Translating...")
        self.state = self.GO

        # translation
        last_diff = inf
        while True:
            self.update_pose()
            current_pose = self.get_pose()
            dx = next_pose.x - current_pose.x
            dy = next_pose.y - current_pose.y
            theta = atan2(dy, dx)
            diff = sqrt((dx**2) + (dy**2))
            # print("#DEBUG - moving pose ({}, {}, {}) - theta: {} - beta: {} - diff: {}".format(current_pose.x, current_pose.y, current_pose.yaw, theta, beta, diff))
            if last_diff < (diff - 0.01) or diff < 0.5:
                self.send_twist(0, 0)
                rospy.sleep(rospy.Duration(3))
                break
            else:
                self.send_twist(self.linear_speed, 0)
                last_diff = diff
            rospy.sleep(rospy.Duration(0.5))
                
        # stop
        self.state = self.STOP
        self.update_pose()
        current_pose = self.get_pose()
        dx = next_pose.x - current_pose.x
        dy = next_pose.y - current_pose.y
        error = sqrt((dx**2) + (dy**2))
        self.update_error(error)
        print("Translation is Done!")
        print("error: {}, avg. error: {}".format(error, self.e_sum/self.e_num))
        return True

    # test robot
    def test(self):
        self.send_twist(0, 0)
        rospy.sleep(rospy.Duration(5))
        self.send_twist(2, 0)
        rospy.sleep(rospy.Duration(3))
        self.send_twist(0, 0)
        rospy.sleep(rospy.Duration(3))
        self.send_twist(-2, 0)              # this does not work (negetive linear)
        rospy.sleep(rospy.Duration(3))
        self.send_twist(0, 0)
        rospy.sleep(rospy.Duration(3))
        self.send_twist(0, 1)
        rospy.sleep(rospy.Duration(3))
        self.send_twist(0, 0)
        rospy.sleep(rospy.Duration(3))
        self.send_twist(0, -1)
        rospy.sleep(rospy.Duration(3))
        self.send_twist(0, 0)
        rospy.sleep(rospy.Duration(3))

    # run controller node
    def Run(self, max_try):
        # self.test()
        print("Max Try: {}".format(max_try))
        rospy.sleep(rospy.Duration(5))
        count = 1
        while count <= max_try:
            print("\n\n\nTry Num: {}\n".format(count))
            self.update_pose()
            next_pose = self.get_next_pose(self.pose.x, self.pose.y)
            if not next_pose:
                # rospy.sleep(rospy.Duration(1))
                continue
            if not self.move(next_pose):
                rospy.sleep(rospy.Duration(1))
                continue
            count += 1

# Run code
if __name__ == "__main__":
    # linear_speed = rospy.get_param('linear_speed')
    # angular_speed = rospy.get_param('angular_speed')

    linear_speed = 4
    angular_speed = 0.1

    ctl = Controller(linear_speed, angular_speed)
    ctl.Run(10)
