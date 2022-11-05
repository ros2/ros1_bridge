FROM ros:noetic as noetic
FROM ros:foxy as foxy
# combine two images
COPY --from=noetic . /


RUN apt-get update -y && \
    apt-get install -y python3-pip python-dev curl lsb-release
RUN pip3 install --upgrade setuptools
RUN python3 -m pip install --upgrade pip
RUN apt-get install git python3-dev python3-catkin-tools python3-numpy libprotobuf-dev protobuf-compiler -y
RUN apt install -y ros-foxy-rosbag2-bag-v2-plugins

#Create sinle ROS 1 source
#_________________________________
WORKDIR /catkin_ws/src
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"
#_________________________________

#Create simple ROS 2 source
#_________________________________
WORKDIR /colcon_ws/src
WORKDIR /colcon_ws/
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"
#_________________________________

#Create Bridge ROS 2
#_________________________________
#if bridge is in project
#COPY . /
# eather copy from local or download from git
WORKDIR /ros1_bridge_ws/src
RUN git clone -b foxy https://github.com/ros2/ros1_bridge.git

WORKDIR /ros1_bridge_ws
RUN /bin/bash -c "source /catkin_ws/devel/setup.bash && source /colcon_ws/install/setup.bash && colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE"
#_________________________________