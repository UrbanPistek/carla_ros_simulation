#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /root/carla-ros-bridge/install/setup.bash

exec "$@"
