#!/usr/bin/env python
import rospy
import math
import sys
import struct
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
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
    r = 124
    g = 252
    b = 0
    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
    #cloud_points = [[0, 0, 0.0, rgb],[1.0, 2.0, 0.0, rgb], [1.0, 3.0, 0.0, rgb]]

    # Input parameters: height and lenght of robot.
    # Input parameters: number of points
    height = 0.3
    lenght = 0.3
    lamba = 0.05
    cloud_points = []
    for x in np.arange(0,height,lamba):
        for y in np.arange(-lenght,lenght,lamba):
            cloud_points.append([x, y, 0.0, rgb])
            cloud_points.append([-x, y, 0.0, rgb])

    '''
    cloud_points = [[0, 0, 0.0, rgb],[0.1, 0.1, 0.0, rgb],[-0.1, -0.1, 0.0, rgb],[0.1, -0.1, 0.0, rgb]
    ,[-0.1, 0.1, 0.0, rgb],[-0.1, 0, 0.0, rgb],[0.1, 0, 0.0, rgb],[0, 0.1, 0.0, rgb],[0, -0.1, 0.0, rgb]]
    '''
    #header
    header = std_msgs.msg.Header()
    
    header.frame_id = 'robot_base_link'
    #create pcl from points
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 12, PointField.UINT32, 1),
          ]

    #scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
    scaled_polygon_pcl = pcl2.create_cloud(header, fields, cloud_points)
    #publish    
    

    while not rospy.is_shutdown():
        rospy.loginfo("happily publishing sample pointcloud.. !")
        scaled_polygon_pcl.header.stamp = rospy.Time.now()
        pcl_pub.publish(scaled_polygon_pcl)

        rospy.sleep(1.0)