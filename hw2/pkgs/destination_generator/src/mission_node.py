#!/usr/bin/python3

import rospy
from destination_generator.srv import GetNextDestination, GetNextDestinationResponse
from random import randint
from math import ceil, sqrt


# Build random next des
def generate_next_des(req):
   x = req.current_x
   y = req.current_y
   dx = randint(0, 10)
   if (25 - (dx*dx)) > 0:
      dy = randint(ceil(sqrt(25 - (dx * dx))), 10)
   else:
      dy = randint(0, 10)
   if randint(0, 1) == 1:
      dx = -1 * dx
   if randint(0, 1) == 1:
      dy = -1 * dy
   x = x + dx
   y = y + dy
   print("Current:({}, {}) - Next:({}, {})".format(req.current_x, req.current_y, x, y))
   return GetNextDestinationResponse(x, y)

def generate_next_des_server():
    rospy.init_node('mission_node')
    s = rospy.Service('get_next_destination', GetNextDestination, generate_next_des)
    print("Mission Node is running!")
    rospy.spin()
 
if __name__ == "__main__":
   generate_next_des_server()