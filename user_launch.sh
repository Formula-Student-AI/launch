#!/bin/bash
set -e

source /opt/ros/galactic/setup.bash
source /home/bristol-fsai/core-sim/install/setup.bash

# Launch AI system in hardware mode (headless)
echo "Launching ROS scripts"

/home/bristol-fsai/launch/ros_bag_script.sh &
/home/bristol-fsai/launch/ros_script_1.sh &
/home/bristol-fsai/launch/ros_script_2.sh &
/home/bristol-fsai/launch/io_logger.sh &
