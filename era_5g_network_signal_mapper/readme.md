# signal_quality_network_application

1) Generate pointcloud2 with colour schema based on quality of network signal.

  -To create the pointcloud --> reference: https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb
  
 2) To save and load the pointcloud map use pcl_ros:
 
  - pcl_ros --> reference: https://wiki.ros.org/pcl_ros#pointcloud_to_pcd
 
 Parameters of signal mapper pacakge:
 
  -Bounding box of robot: I.E 2x2 meters
 



Run signal mapper with:

´´´
source devel/setup.bash
´´´

´´´
rosrun era_5g_network_signal_mapper signal_mapper.py 
´´´
