# signal_quality_network_application

Code tested under ROS 1 Melodic


Running instructions.

*This code was tested under RB1 Base complete_sim.*

Source workspace
```
source devel/setup.bash
```
Run the robot simulation
```
roslaunch rb1_base_sim_bringup rb1_base_complete.launch 
```

Create current pointcloud around the robot:
```
rosrun era_5g_network_signal_mapper signal_mapper.py 
```

Save each published pointcloud to historical pcl map:

```
rosrun era_5g_network_signal_mapper sub_sigl_mapper.py 
```
![rb1](https://user-images.githubusercontent.com/26432703/235520519-f5acbad8-6d19-420d-935f-e533671cd073.png)

Configure RVIZ to show the pcl2 by adding a pointcloud2 object and add the **/semantic_pcl** topic. Also adjust the size of the particle to 0.05 for better visualization.

* Remote control the robot:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot/cmd_vel
```
Change colour of current pointcloud:
```
rosrun era_5g_network_signal_mapper colour_pub.py 
```

 ## To save and load the pointcloud map use pcl_ros:
 
 This command will save the pointcloud2 into a pcd file. In has as input parameter the topic where the pointcloud2 map is published. 
 
```
rosrun pcl_ros pointcloud_to_pcd input:=/semantic_pcl
```

To publish the pointcloud again from the pcd file under a particular frame:

```
rosrun pcl_ros pcd_to_pointcloud 2240726000.pcd 0.1 _frame_id:=/robot_map rosrun pcl_ros pcd_to_pointcloud 2240726000.pcd 0.1 _frame_id:=/robot_map
```

Configure rviz to show the pointcloud2, make sure to increase the size of the particles for visualization.

[reference](https://wiki.ros.org/pcl_ros#pointcloud_to_pcd)

## From pcl & current map to new occupancy grid map:
 
```
rosrun era_5g_network_signal_mapper costmap_translate.py
```

Result looks like this, an occupancyGrid map is created with the data of the current map /map and the semantic pcl2. In this package, the filter of objects that should not be obstacles can be done by reading the pcl2 colour and drop the points before creating the new map.

![pcl_to_grid](https://user-images.githubusercontent.com/26432703/235369752-40e69d1f-221b-4603-b649-3e1253b6b55e.png)

You may use map server map saver to save the map. Later on, in the launch file for the robot, in the mapserver section, add the new map. This will allow the robot to navigate taking into consideration the newly created pcl2 obstacles.



 
