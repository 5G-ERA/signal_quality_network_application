# signal_quality_network_application

Running instructions.

*This code assumes RB1 Base complete_sim is running.*

Source workspace
```
source devel/setup.bash
```

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

Change colour of current pointcloud:
```
rosrun era_5g_network_signal_mapper colour_pub.py 
```

 ## To save and load the pointcloud map use pcl_ros:
 
  pcl_ros --> reference: https://wiki.ros.org/pcl_ros#pointcloud_to_pcd

 
