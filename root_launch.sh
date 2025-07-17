#!/bin/bash
set -e

echo "----------------------"

echo "Ensuring permissions and folders are setup"
mkdir /home/bristol-fsai/logs &
sudo chmod +x /home/bristol-fsai/launch/can_launch.sh
sudo chmod +x /home/bristol-fsai/launch/user_launch.sh
sudo chmod +x /home/bristol-fsai/launch/ros_script_1.sh
sudo chmod +x /home/bristol-fsai/launch/ros_script_1.sh

# Enable NVIDIA driver
echo "Enabling NVIDIA drivers"
sudo prime-select nvidia
sudo modprobe nvidia
echo "Enabled NVIDIA drivers"

# Enable CAN modules
echo "Enabling can modules"
sudo modprobe can_dev
echo "Enabled can dev"
sudo modprobe can
echo "Enabled can"
sudo modprobe can_raw
echo "Enabled can raw"
sudo modprobe vcan
echo "Enabled vcan"

# Setup CAN link
sudo /home/bristol-fsai/launch/can_launch.sh &

# Launch user commands
echo "Launching user commands"
su - bristol-fsai -c /home/bristol-fsai/launch/user_launch.sh

