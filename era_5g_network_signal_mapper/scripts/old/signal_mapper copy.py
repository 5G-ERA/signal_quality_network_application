#!/usr/bin/env python
import rospy
import math
import sys

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    rospy.init_node('pcl2_pub_example')
    pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2)
    rospy.loginfo("Initializing sample pcl2 publisher node...")
    #give time to roscore to make the connections
    rospy.sleep(1.)
    cloud_points = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]
    #header
    header = std_msgs.msg.Header()
    
    header.frame_id = 'robot_base_link'
    #create pcl from points
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
    #publish    
    

    while not rospy.is_shutdown():
        rospy.loginfo("happily publishing sample pointcloud.. !")
        scaled_polygon_pcl.header.stamp = rospy.Time.now()
        pcl_pub.publish(scaled_polygon_pcl)

        rospy.sleep(1.0)