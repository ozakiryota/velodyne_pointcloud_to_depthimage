#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2

import math

class VelodynePointcloudToDepthimage:
    def __init__(self):
        ## parameter
        self.num_ring = rospy.get_param("/num_ring", 32)
        print("self.num_ring = ", self.num_ring)
        self.points_per_ring = rospy.get_param("/points_per_ring", 1092)
        print("self.points_per_ring = ", self.points_per_ring)
        self.depth_resolution = rospy.get_param("/depth_resolution", 0.1)
        print("self.depth_resolution = ", self.depth_resolution)
        ## subscriber
        self.sub_pc = rospy.Subscriber("/velodyne_points", PointCloud2, self.callbackPC)
        ## publisher
        self.pub_pc = rospy.Publisher("/depth_imgae", Image, queue_size=1)
        ## msg
        self.img_msg = Image()
        ## cv_bridge
        self.bridge = CvBridge()
        ## copy arguments

    def callbackPC(self, msg):
        # point = next(iter(pc2.read_points(msg)))
        # print(point)
        points = msg
        pcToImage(points)

    def pcToImage(self, points):


    def publication(self, stamp):
        self.pub_pc.publish(self.img_msg)

def main():
    rospy.init_node('velodyne_pointcloud_to_depthimage', anonymous=True)

    velodyne_pointcloud_to_depthimage = VelodynePointcloudToDepthimage()

    rospy.spin()

if __name__ == '__main__':
    main()
