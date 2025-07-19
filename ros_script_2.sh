#!/bin/bash
set -e

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 >> "/home/bristol-fsai/logs/ros2_$(date +%Y-%m-%d_%H-%M-%S).log" 2>&1
