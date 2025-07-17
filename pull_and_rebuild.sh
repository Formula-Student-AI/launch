#!/bin/bash

cd ~/core-sim
git pull
source install/setup.bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

echo "all done :)"
