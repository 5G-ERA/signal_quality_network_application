



Running instructions.

*This code assumes RB1 Base complete_sim or robot is running.*

Download docker image of signal mapper and install container in interactive mode with environment; and give expicit name to container "signal_mapper_container_ros1", not mandatory and name can be assigned as you want.
```
docker run --network host --name signal_mapper_container_ros1 -it appmdeve/signal_mapper_ros1
```

For oppening another terminal with same container run: (assumes that container is active.)
```
docker exec -it signal_mapper_container_ros1 bash
```

For relaunching container run:   (assumes that container was instaled previously)

```
docker start -i signal_mapper_container_ros1
```

Run signal_mapper launch with customized parameters:

```
roslaunch era_5g_network_signal_mapper signal_mapper.launch map_frame:=map semantic_map_frame:=semantic_map base_link:=robot_base_link
```

Or run signal_mapper launch with default parameters: 
```
roslaunch era_5g_network_signal_mapper signal_mapper.launch
```
Open another terminal and enter in the "signal_mapper_container_ros1"
```
docker exec -it signal_mapper_container_ros1 bash
```

Connect to the InfluxDB and publish colors accordingly to signal with customized params of this connection:

client = InfluxDBClient('192.168.23.245', 8086, 'test5g', 'TEST2023', 'openwrt')

client = InfluxDBClient(host, port, username, password, database)

```
roslaunch era_5g_network_signal_mapper color_publisher.launch host:=localhost port:=8086 username:=test5g password:=TEST2023 database:=openwrt
```

Alternative run generic color publisher for demo purpose:

Open another terminal and enter in the "signal_mapper_container_ros1"
```
docker exec -it signal_mapper_container_ros1 bash
```
Run generic color publisher:
```
rosrun era_5g_network_signal_mapper colour_pub.py
```

For publishing costmap_translate execute following:

Open another terminal and enter in the "signal_mapper_container_ros1"
```
docker exec -it signal_mapper_container_ros1 bash
```

Run costmap_translate launch for publishing created map with customized parameters
```
roslaunch era_5g_network_signal_mapper costmap_translate.launch map_metadata_topic:=/map_metadata map_topic:=/map map_frame:=robot_map
```

