# Use the official ROS Melodic base image
FROM osrf/ros:melodic-desktop-full

# TODO

# # Set the locale
# RUN apt-get update && apt-get install -y locales && \
#     locale-gen en_US.UTF-8 && \
#     update-locale LANG=en_US.UTF-8

# ENV LANG en_US.UTF-8

# # Install development tools and ROS packages
# RUN apt-get update && apt-get install -y \
#     python-rosinstall python-rosinstall-generator python-wstool build-essential \
#     ros-melodic-navigation ros-melodic-gmapping ros-melodic-move-base \
#     ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-moveit \
#     ros-melodic-gazebo-plugins ros-melodic-smach ros-melodic-flexbe-behavior-engine \
#     ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-rospy && \
#     rm -rf /var/lib/apt/lists/*

# # Setup catkin workspace
# RUN mkdir -p /catkin_ws/src
# WORKDIR /catkin_ws

# # Initialize the catkin workspace
# RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

# # Copy your project files into the Docker container
# COPY ./src /catkin_ws/src

# # Build the catkin workspace
# RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

# # Source the workspace in every new shell
# RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# # Expose any ports you need, for example, for ROS, Gazebo, or other services
# EXPOSE 11311

# # Default command to run on container start-up
# # CMD ["roslaunch", "your_package your_launch_file.launch"]
