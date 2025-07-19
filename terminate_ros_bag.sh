#!/bin/bash

kill -SIGINT $(pgrep -f "ros2 bag record")
