#!/bin/bash
# Isaac Sim 5.1 (Python 3.11) must not see the system ROS 2 Jazzy (Python 3.12).
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '^/opt/ros' | paste -sd ':')

ISAAC=$HOME/isaacsim
ROS_LIBS=$ISAAC/exts/isaacsim.ros2.bridge/jazzy/lib
if [ ! -d "$ROS_LIBS" ]; then
  echo "Can't find Isaac's internal ROS2 libs at $ROS_LIBS"
  exit 1
fi

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_LIBS
cd "$(dirname "$0")"
exec $ISAAC/python.sh isaac_cell.py
