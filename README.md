# ROS project

## Manual

```sh
ipconfig # IPv4 Address. . . . . . . . . . . : 172.26.240.1

# Start docker container
docker run --rm -it -e DISPLAY=172.26.240.1:0.0 --mount type=bind,source="C:/Users/soura/Desktop/UTD courses/FALL24/CS6301/CS6301-project/workspace",target=/home/ros/workspace --name ros-container osrf/ros:noetic-desktop-full

# Setup workspace
apt update
cd /home/ros/workspace
source /opt/ros/noetic/setup.bash
mkdir src
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH # /home/ros/workspace/src:/opt/ros/noetic/share

# Clone dependencies
sudo apt install git -y
git clone -b gazebo11 git@github.com:ZebraDevs/fetch_gazebo.git
mv fetch_gazebo/fetch_gazebo ./src
rm -rf fetch_gazebo
apt install -y ros-noetic-robot-controllers ros-noetic-rgbd-launch ros-noetic-fetch-description

# Remake
catkin_make
source devel/setup.bash
roslaunch fetch_gazebo simple_grasp.launch

# Rviz in another terminal
docker exec -it ros-container bash
cd /home/ros/workspace
source devel/setup.bash
rosrun rviz rviz

# Python in 3rd terminal
docker exec -it ros-container bash
cd /home/ros/workspace
source devel/setup.bash
sudo apt update && sudo apt install python3 python3-pip -y
pip3 install numpy transforms3d
sudo ln -s /usr/bin/python3 /usr/bin/python
```

## Dockerfile

```sh
docker build -t ros-project .

docker run -it --rm \
    -e DISPLAY=host.docker.internal:0.0 \
    -v C:/path/to/your/project/src:/catkin_ws/src \
    ros-project
```
