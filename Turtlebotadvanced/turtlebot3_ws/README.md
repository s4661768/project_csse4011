TURTLEBOT README

This file would detail how to successfully compile and and flash the Turtlebot code for this project. It is assumed the user has a running ros2 workspace.

first download the following packages.


cd ~
git clone https://github.com/s4661768/project_csse4011.git
cd project_csse4011
colcon build
source install/setup.bash

echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc

pip uninstall opencv-python -y
pip uninstall opencv-contrib-python -y
pip install opencv-contrib-python==4.6.0.66



To begin, in seperate terminals call the following. 

ros2 launch nav2_bringup navigation_launch.py

ros2 launch slam_toolbox online_async_launch.py

ros2 run gesture_cont gest_node

