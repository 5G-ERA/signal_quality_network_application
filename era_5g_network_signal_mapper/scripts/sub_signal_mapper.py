#!/usr/bin/env python3
import rospy
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

    'Static variables'
    map_frame = rospy.get_param('/sub_signal_mapper/map_frame') # By default --> /robot_map
    semantic_map_frame = rospy.get_param('/sub_signal_mapper/semantic_map_frame') # By default --> /robot_map /semantic_map

    'historical pcl2'
    merged_cloud = []

    'Function to construct a pointcloud2 object'
    def construct_pointCloud2(points):

        header = std_msgs.msg.Header()
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            ]

        scaled_polygon_pcl = pcl2.create_cloud(header, fields, points)
        scaled_polygon_pcl.header.stamp = rospy.Time.now()
        scaled_polygon_pcl.header.frame_id = map_frame
        return scaled_polygon_pcl

    def sub_callback(pcl):
        
        tf.waitForTransform("/"+map_frame, "/"+semantic_map_frame, rospy.Time(), rospy.Duration(4.0))
        t = tf.getLatestCommonTime(map_frame, "/"+semantic_map_frame) # Transform from /semantic_map frame to /robot_map frame.

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        trans = tf_buffer.lookup_transform(map_frame, semantic_map_frame,rospy.Time(), rospy.Duration(4.0))
        cloud_out = do_transform_cloud(pcl, trans)
        
        # Get the points from the pcl2.
        gen = pcl2.read_points(cloud_out, skip_nans=True)
        int_data = list(gen)

        temp_list = []
        for element in int_data:
            mini = []
            for x in element:
                
                mini.append(x)
            temp_list.append(mini)

        # Append latest pointcloud to merged_cloud
        for y in temp_list:
            merged_cloud.append(y)
        #merged_cloud.append(temp_list) # no es append, es otra cosa parecida.

        # Recreate the merged pointcloud
        map_pcl = construct_pointCloud2(merged_cloud) 

        semantic_pcl_pub.publish(map_pcl)
        rospy.sleep(1.0)

    '''ROS node definition'''
    
    rospy.init_node('pcl2_semantic_mapper')
    br = tf.TransformBroadcaster() #Define fixed frame for semantic map.
    semantic_pcl_sub = rospy.Subscriber("/current_semantic_pcl", PointCloud2, sub_callback, queue_size=1, buff_size=542428800 )
    semantic_pcl_pub = rospy.Publisher("/semantic_pcl", PointCloud2)
    tf = TransformListener()
    rospy.spin()
    
