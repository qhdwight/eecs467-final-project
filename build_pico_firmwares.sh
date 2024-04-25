#/bin/bash

# Build projects
cd ~/catkin_ws/src/hockey_cup/src/pico-drive-controller/build/
make -j4
echo "------------------------------------------------------------"
cd ~/catkin_ws/src/hockey_cup/src/pico-ball-controller/build/
make -j4
