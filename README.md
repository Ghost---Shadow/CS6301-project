# ROS project

```sh
docker build -t ros-project .

docker run -it --rm \
    -e DISPLAY=host.docker.internal:0.0 \
    -v C:/path/to/your/project/src:/catkin_ws/src \
    ros-project
```
