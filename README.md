# Soup Operator

## Environment setup (Powershell)

```powershell
ipconfig # IPv4 Address. . . . . . . . . . . : 172.26.240.1

# Define variables
$HOST_IP = "172.26.240.1"  # Get from ipconfig
$LOCAL_WORKSPACE = "${PWD}\workspace"
$CONTAINER_WORKSPACE = "/home/ros/workspace"
$CONTAINER_NAME = "ros-container"
$IMAGE_NAME = "osrf/ros:noetic-desktop-full"

# Start Docker container
docker run --rm -it -e DISPLAY=$HOST_IP`:0 --mount type=bind,source=`"$LOCAL_WORKSPACE`",target=$CONTAINER_WORKSPACE --name $CONTAINER_NAME $IMAGE_NAME

# Prepare the workspace
cd /home/ros/workspace
source setup.sh
```

## Running simulation

### Terminal 1

```sh
# Already running terminal
roslaunch fetch_moveit_config move_group.launch
```

### Terminal 2

```sh
docker exec -it ros-container bash
cd /home/ros/workspace
source devel/setup.bash

# OpenAI token
# Get one from here https://platform.openai.com/api-keys
# Make sure you have money in your account
export OPENAI_API_KEY="<secret>"

# https://github.com/ros-planning/navigation/issues/1125#issuecomment-1238647110
rosrun fetch_gazebo planning.py 2> >(grep -v TF_REPEATED_DATA buffer_core)

# Example queries
# I want a vegan soup
# I want a fish soup
```

## Adding more poses

```sh
# https://www.youtube.com/watch?v=DZB5_4JCS0A&list=PLeEzO_sX5H6TBD6EMGgV-qdhzxPY19m12&index=13
roslaunch moveit_setup_assistant setup_assistant.launch
```
