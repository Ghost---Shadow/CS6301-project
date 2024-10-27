# Apt install everything
apt update && apt install -y ros-noetic-robot-controllers ros-noetic-rgbd-launch ros-noetic-fetch-description git ros-noetic-moveit ros-noetic-grid-map-costmap-2d ros-noetic-trac-ik python3 python3-pip

source /opt/ros/noetic/setup.bash

# Link python and install libraries
sudo ln -s /usr/bin/python3 /usr/bin/python
pip3 install numpy transforms3d openai pydantic

rosdep install --from-paths src --ignore-src -r -y

catkin_make

source devel/setup.bash
