#!/usr/bin/env python3

import rospy 
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

if __name__ == "__main__":

    rospy.init_node("pcd_view_node")
    rospy.wait_for_service("assemble_scans2")
    assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
    pub = rospy.Publisher ("/laser_pointcloud", PointCloud2, queue_size = 10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            res = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
            pub.publish (res.cloud)
        except rospy.ServiceException:
            print("scanning failed!")
        rate.sleep()