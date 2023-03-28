#!/usr/bin/env python
import rospy
import math
import sys
import struct
import numpy as np
import roslib
import tf
from random import randrange
from tf import TransformListener
import tf2_ros
import tf2_py as tf2

from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud



if __name__ == '__main__':

    '''Input parameters'''
    robot_base_frame = 'robot_base_link'
    height = 0.3
    lenght = 0.3
    lamba = 0.05

    'Static variables'
    final_semantic_map_frame = 'final_semantic_map'
    map_fixed_frame = 'robot_map'

    def sub_callback(pcl):

        # Have to append to previous pointclouds received.
        
        tf.waitForTransform("/robot_map", "/semantic_map", rospy.Time(), rospy.Duration(4.0))
        t = tf.getLatestCommonTime("/robot_map", "/semantic_map") # Transform from /semantic_map frame to /robot_map frame.
        '''
        position, quaternion = tf.lookupTransform("/robot_map", "/semantic_map", t)
        print(position)
        print(quaternion)
        print("============")
        
        '''
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        trans = tf_buffer.lookup_transform("robot_map", "semantic_map",rospy.Time(), rospy.Duration(4.0))
        cloud_out = do_transform_cloud(pcl, trans)
        
        '''
        gen = pcl2.read_points(pcl, skip_nans=True, field_names=("x", "y", "z"))
        int_data = list(gen)
        middleIndex = (len(int_data)-1)/2
        middle_point = int_data[middleIndex]
        '''

        '''
        print(pcl.header)
        print(pcl.fields)
        print(int_data)
        print("================")
        '''
        '''
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         final_semantic_map_frame,
                         map_fixed_frame)
        '''
        #pcl.header.frame_id = map_fixed_frame #final_semantic_map_frame
        semantic_pcl_pub.publish(cloud_out)
        rospy.sleep(1.0)
        # = semantic_map_frame

    '''ROS node definition'''
    
    rospy.init_node('pcl2_semantic_mapper')
    br = tf.TransformBroadcaster() #Define fixed frame for semantic map.
    semantic_pcl_sub = rospy.Subscriber("/current_semantic_pcl", PointCloud2, sub_callback, queue_size=1, buff_size=542428800 )
    semantic_pcl_pub = rospy.Publisher("/semantic_pcl", PointCloud2)
    tf = TransformListener()
    rospy.spin()
    