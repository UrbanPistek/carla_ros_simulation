#!/bin/bash

# clone the ROS 2 bridge
mkdir -p ~/carla-ros-bridge 
cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge

source /opt/ros/foxy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r
colcon build

# Install CARLA python api
pip install numpy
pip install pygame
pip install carla

 source ./install/setup.bash
