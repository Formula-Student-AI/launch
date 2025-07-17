#!/bin/bash

echo "Setting up can link"
sudo ip link set up can0 type can bitrate 500000

echo "Starting candump to log CAN traffic"
stdbuf -o0 candump -ta -l /home/bristol-fsai/logs/candump_log_$(date +%Y%m%d_%H%M%S).log can0 &