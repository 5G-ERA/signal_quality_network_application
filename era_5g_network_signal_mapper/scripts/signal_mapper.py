#!/usr/bin/env python3
import rospy
import math
import sys
import struct
import numpy as np
import roslib
import tf

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

if __name__ == '__main__':

    '''Input parameters'''
    
    robot_base_frame = rospy.get_param('/signal_mapper/base_link') # By default use --> 'robot_base_link'
    rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('base_link'), robot_base_frame)

    height = 0.3
    lenght = 0.3
    lamba = 0.05

    'Static variables'
    semantic_map_frame = rospy.get_param('/signal_mapper/semantic_map_frame') # By default use --> 'semantic_map'
    
    #Give colour to the pointcloud around the robot.
    r = 124 # 
    g = 252 #
    b = 0

    '''ROS node definition'''
    br = tf.TransformBroadcaster() #Define fixed frame for semantic map.
    rospy.init_node('pcl2_pub_example')

    current_pcl_pub = rospy.Publisher("/current_semantic_pcl", PointCloud2)
    rospy.sleep(1.)

    '''Set pcl colour callback'''
    def callback(data):
        rospy.loginfo(data.data)
        global r
        global g
        global b
        if str(data.data) == "GREEN":
            
            r = 124
            g = 252
            b = 0

        if str(data.data) == "RED":

            r = 255
            g = 0
            b = 0

        if str(data.data) == "BLUE":

            r = 128
            g = 0
            b = 0

    rospy.Subscriber("pcl_colour", String, callback)

    rospy.loginfo("Initializing semantic pcl2 mapper...")

    def create_complex_pointcloud():

        cloud_points = []
        for x in np.arange(0,height,lamba):
            for y in np.arange(-lenght,lenght,lamba):
                cloud_points.append([x, y, 0.0, rgb])
                cloud_points.append([-x, y, 0.0, rgb])
        return cloud_points

    '''
    cloud_points = [[0, 0, 0.0, rgb],[0.1, 0.1, 0.0, rgb],[-0.1, -0.1, 0.0, rgb],[0.1, -0.1, 0.0, rgb]
    ,[-0.1, 0.1, 0.0, rgb],[-0.1, 0, 0.0, rgb],[0.1, 0, 0.0, rgb],[0, 0.1, 0.0, rgb],[0, -0.1, 0.0, rgb]]
    '''

    def create_simple_pointcloud():

        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
        #cloud_points = [[0, 0, 0.0, rgb]]
        #cloud_points = [[0, 0, 0.0, rgb],[0.1, 0.1, 0.0, rgb]]
        
        cloud_points = []
        for x in np.arange(0,height,lamba):
            for y in np.arange(-lenght,lenght,lamba):
                cloud_points.append([x, y, 0.0, rgb])
                cloud_points.append([-x, y, 0.0, rgb])
        
        header = std_msgs.msg.Header()
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            ]

        #scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
        scaled_polygon_pcl = pcl2.create_cloud(header, fields, cloud_points)
        scaled_polygon_pcl.header.stamp = rospy.Time.now()
        scaled_polygon_pcl.header.frame_id = semantic_map_frame
        current_pcl_pub.publish(scaled_polygon_pcl)
        rospy.sleep(1.0)
    
    # robot_map
    while not rospy.is_shutdown():
        print(r,g,b)
        
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         semantic_map_frame,
                         robot_base_frame)
        create_simple_pointcloud()
