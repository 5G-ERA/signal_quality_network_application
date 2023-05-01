#!/usr/bin/env python
import rospy
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from enum import Enum

# List of colours from the pcl2 that should not be treated as obstacles by the ROS Navigation Stack.
ACCEPTED_COLOURS = []

class Colour():

     def __init__(self, rgb ):
        self.rgb = rgb
        self.binary = struct.unpack('I', struct.pack('BBBB', rgb[2], rgb[1], rgb[0], 0))[0]

Green = Colour([124, 252, 0])
Red = Colour([255, 0, 0])
Blue = Colour([128,0,0])

ACCEPTED_COLOURS.append(Green.binary)
print(Green.binary)

class Map(object):

    def __init__(self, origin_x=-50, origin_y=-50, resolution=0.02, 
                 width=50, height=50):

        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        self.grid = np.full((height, width), -1.)
    

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

        self.flat_grid = self.grid.reshape((self.grid.size,))
        self.final = list(np.round(self.flat_grid))
        self.grid_msg.data = self.final
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
    
    def __init__(self):

        '''ROS node definition'''
        rospy.init_node('pcl2_to_costmap')
        # Subcribers
        metadata = rospy.wait_for_message("/robot/map_metadata", MapMetaData, timeout=5)

        self.res = metadata.resolution
        self.w = metadata.width
        self.h = metadata.height
        self.o = metadata.origin

        self._map = Map(self.o.position.x, self.o.position.y, self.res, self.w, self.h)
        map = rospy.wait_for_message("/robot/map", OccupancyGrid, timeout=5)
        self.grid_msg = self._map.toMap(map.data)
        rospy.Subscriber("/semantic_pcl", PointCloud2, self.pcl_callback, queue_size=1, buff_size=542428800 )
        

        # Publishers
        self._map_pub = rospy.Publisher("/map_Semantic", OccupancyGrid, latch=True,queue_size=1 ) # To get the metadata for dimension, origin and so own to repeat in the semantic_costmap_pcl
        self._map_metadata_pub = rospy.Publisher('/map_Semantic_metadata', MapMetaData, latch=True, queue_size=1)
        
        rospy.loginfo("Publishing semantic map.")
        rospy.spin()

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
                arr = arr.data
        grid.data = arr.ravel()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "robot_map" 
        grid.info = info
 
        return grid
        
    
    def pcl_callback(self, pointcloud_msg):
        pcl_cloud = list(pcl2.read_points(pointcloud_msg, skip_nans=True))
        # To manipulate the OccupancyGrid and add the pcl2 data, first it needs to be translated into a numpy MaskedArray.
        np_ocupancy = self.occupancygrid_to_numpy(self.grid_msg)
        
        try:
            for point in pcl_cloud:
                #print(point[3])
                # Only pcl points with colour not in the accepted array will be considered obstacles and added to the grid with prob(100)
                if point[3] not in ACCEPTED_COLOURS:
                    print("holi")
                    y = int((point[0] - self._map.origin_x) / self._map.resolution) 
                    x = int((point[1]*50 + self._map.origin_y*1+-40) +11/ self._map.resolution *1)
                    np_ocupancy[x][y] = 100.

            map_OccupancyGrid = self.numpy_to_occupancy_grid(np_ocupancy, self.grid_msg.info)
            self._map_metadata_pub.publish(self.grid_msg.info)
            self._map_pub.publish(map_OccupancyGrid)
        except Exception as e:
            print(e)
        

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

        # Create map
        self._map = Map(o.position.x, o.position.y, res, w, h)
        rospy.loginfo("Reading metadata of original map...")


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass
    

   
