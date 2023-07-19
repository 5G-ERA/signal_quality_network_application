FROM osrf/ros:noetic-desktop

RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN pip3 install influxdb
RUN pip3 install influxdb-client


RUN apt-get install -y ros-noetic-tf2-geometry-msgs
RUN apt-get install -y ros-noetic-geometry2

RUN mkdir -p ~/catkin_ws/src
RUN cd ~/catkin_ws

WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
catkin_make"

COPY src src

#RUN rosdep install -i --from-path src --rosdistro noetic -y

# Source the workspace setup file
RUN /bin/bash -c "source /root/catkin_ws/devel/setup.bash"

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc