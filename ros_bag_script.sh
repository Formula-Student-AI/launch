#!/bin/bash

cd ~/logs

ros2 bag record -o "ros_bag_$(date +%Y-%m-%d_%H-%M-%S)" /perception/cones /pf_cones /ros_can/twist /ros_can/wheel_speeds /ros_can/vehicle_commands /cmd /spline_trajectory stop_zone_marker

