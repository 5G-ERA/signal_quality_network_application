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
import time
import tf2_py as tf2
import pcl_ros

from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan

FILTERED_COLOURS = [(124, 252, 0), (255, 0, 0)] # green, red

class Map(object):

    def __init__(self, origin_x=-50, origin_y=-50, resolution=0.02, 
                 width=50, height=50):

        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        #self.grid = np.zeros((height, width))
        self.grid = np.full((height, width), -1.)
        #self.grid = np.zeros((50, 50))

        # -1 unkown
        # 0 free
        # 1 occupayed
        
    

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        self.grid_msg = OccupancyGrid()

        # Set up the header.
        self.grid_msg.header.stamp = rospy.Time.now()
        self.grid_msg.header.frame_id = "robot_map" # robot_map

        # .info is a nav_msgs/MapMetaData message. 
        self.grid_msg.info.resolution = self.resolution
        self.grid_msg.info.width = self.width
        self.grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        self.grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).

        self.flat_grid = self.grid.reshape((self.grid.size,)) #* 100
        #self.final = list(np.round(self.flat_grid))
        self.final = list(np.round(self.flat_grid))
        #print(self.final)
        
        #self.grid_msg.data = 
        self.grid_msg.data = self.final
        # 4992
       
        return self.grid_msg

    def toMap(self, map_data):

        self.grid_msg = OccupancyGrid()

        # Set up the header.
        self.grid_msg.header.stamp = rospy.Time.now()
        self.grid_msg.header.frame_id = "robot_map" # robot_map

        # .info is a nav_msgs/MapMetaData message. 
        self.grid_msg.info.resolution = self.resolution
        self.grid_msg.info.width = self.width
        self.grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        self.grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))
        self.grid_msg.data = map_data
        return self.grid_msg
    
class Mapper(object):
    """ 
    The Mapper class creates a map from laser scan data.
    """
    
    def __init__(self):
        """ Start the mapper. """

        '''ROS node definition'''
        rospy.init_node('pcl2_to_costmap')
        # Subcribers
        '''

        scan_map = rospy.Subscriber("/robot/front_laser/scan", LaserScan, self.scan_callback, queue_size=1)
        self.rate = rospy.Rate(30) # ROS Rate at 5Hz
        '''
        #map_metadata_sub = rospy.Subscriber("/robot/map_metadata", MapMetaData, self.map_metadata_callback, queue_size=1, buff_size=542428800 )
        metadata = rospy.wait_for_message("/robot/map_metadata", MapMetaData, timeout=5)

        self.res = metadata.resolution
        self.w = metadata.width
        self.h = metadata.height
        self.o = metadata.origin

        # Create map
        self._map = Map(self.o.position.x, self.o.position.y, self.res, self.w, self.h)

        #map_sub = rospy.Subscriber("/robot/map", OccupancyGrid, self.map_callback, queue_size=1, buff_size=542428800 )
        map = rospy.wait_for_message("/robot/map", OccupancyGrid, timeout=5)
        self.grid_msg = self._map.toMap(map.data)
        
        semantic_pcl_sub = rospy.Subscriber("/cloud_pcd", PointCloud2, self.pcl_callback, queue_size=1, buff_size=542428800 )

        # Publishers
        #semantic_costmap_pcl_pub = rospy.Publisher("/semantic_OccupancyGrid", OccupancyGrid)
        self._map_pub = rospy.Publisher("/map_Semantic", OccupancyGrid, latch=True,queue_size=1 ) # To get the metadata for dimension, origin and so own to repeat in the semantic_costmap_pcl
        self._map_metadata_pub = rospy.Publisher('/map_Semantic_metadata', MapMetaData, latch=True, queue_size=1)
        
        rospy.loginfo("Publishing semantic map.")
        '''
        self._map = Map()
        self.grid_msg = self._map.to_message()
        self._map_metadata_pub.publish(self.grid_msg.info)
        self._map_pub.publish(self.grid_msg)
        '''
        rospy.spin()

    def scan_callback(self, scan):
 
        self.publish_map()

   # reference: http://docs.ros.org/en/jade/api/ros_numpy/html/occupancy__grid_8py_source.html
    def occupancygrid_to_numpy(self,msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        return np.ma.array(data, mask=data==-1, fill_value=-1)
    
    def numpy_to_occupancy_grid(self, arr, info=None):
        if not len(arr.shape) == 2:
                 raise TypeError('Array must be 2D')
        if not arr.dtype == np.int8:
                 raise TypeError('Array must be of int8s')
 
        grid = OccupancyGrid()
        if isinstance(arr, np.ma.MaskedArray):
                 # We assume that the masked value are already -1, for speed
                arr = arr.data
        grid.data = arr.ravel()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "robot_map" 
        grid.info = info
 
        return grid
        
    
    def pcl_callback(self, pointcloud_msg):
        pcl_cloud = list(pcl2.read_points(pointcloud_msg, skip_nans=True))

        #self._map_metadata_pub.publish(self.grid_msg.info)
        #self._map_pub.publish(self.grid_msg)
        np_ocupancy = self.occupancygrid_to_numpy(self.grid_msg)
        
        for x in range(0,100):
             for y in range(0,100):
                np_ocupancy[y][x] = 100.

        print(np_ocupancy[0][0])
        #print(type(np_ocupancy))
        

        
        
       
        try:
            for point in pcl_cloud:
                #print(point)
                #print("==========")
               # x = int((point[0]*5 + self._map.origin_x *1-22) +8.5/ self._map.resolution*1)
                y = int((point[0] - self._map.origin_x) / self._map.resolution) 
                x = int((point[1]*50 + self._map.origin_y*1+-40) +11/ self._map.resolution *1)
                '''
                x = int((point[0] - self._map.origin_x) / self._map.resolution) 
                
                y = int((point[1] - self._map.origin_y)  / self._map.resolution) 
                
                print("origin x "+str(self._map.origin_x))
                print("origin y "+str(self._map.origin_y))
                x = int(point[0])
                x = int(point[1])
                '''
                #print(self.grid_msg.data[y * 1000 + x])
                

                np_ocupancy[x][y] = 100.
                #np_ocupancy[y * 1000 + x] = 100.
                

                #self.grid_msg.data = 0

                #self.grid_msg.data = self.grid_msg.data
               
                #if x >= 0 and x < self._map.width and y >= 0 and y < self._map.height:
                    
                    #exit(0)
                    #self.grid_msg.data[y * self._map.width + x] = 1.

            map_OccupancyGrid = self.numpy_to_occupancy_grid(np_ocupancy, self.grid_msg.info)
            self._map_metadata_pub.publish(self.grid_msg.info)
            self._map_pub.publish(map_OccupancyGrid)
        except Exception as e:
            print(e)
        
    def publish_map(self):
        pass

    def map_callback(self, msg):
        print(len(msg.data))
        try:
            self.grid_msg = self._map.toMap(msg.data)
            self._map_metadata_pub.publish(self.grid_msg.info)
            self._map_pub.publish(self.grid_msg)
        except Exception as a:
            print(a)
        

    '''Only called once'''
    def map_metadata_callback(self, data):
        res = data.resolution
        w = data.width
        h = data.height
        o = data.origin

        print("resolution: "+str(res))
        print("width: "+str(w))
        print("height: "+str(h))
        #print("origin: "+str(o))
        print("origin pos:"+str(o.position))
        print("origin orientation:"+str(o.orientation))

        # Create map
        self._map = Map(o.position.x, o.position.y, res, w, h)
        rospy.loginfo("Create empty map.")
        '''
        self.grid_msg = self._map.to_message()
        self._map_metadata_pub.publish(self.grid_msg.info)
        self._map_pub.publish(self.grid_msg)
        '''
        # Populate map
        '''
        self._map.grid[0, 0] = 1.0
        '''

        
        
        

# map_msgs/OccupancyGridUpdate
# /robot/map_updates


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass
    

    
        

    
    