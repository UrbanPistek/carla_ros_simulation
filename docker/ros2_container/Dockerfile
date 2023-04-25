# First stage sets up the base, ie everything except the active dev repo
FROM  osrf/ros:foxy-desktop-focal AS base

# Setup ROS 2 Workspace
SHELL ["/bin/bash", "-c"]

RUN sudo apt install python3-colcon-common-extensions

# RUN mkdir /ros2_ws
COPY ros2_ws ros2_ws 

RUN cd ros2_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /

# Ensure script is executable
RUN ["chmod", "+x", "/ros_entrypoint.sh"]

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
