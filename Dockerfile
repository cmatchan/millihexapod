# syntax=docker/dockerfile:1

# USAGE: DOCKER_BUILDKIT=1 docker build -t cmatchan/ros-noetic-millihex .
#        docker run --name container_name -it cmatchan/ros-noetic-millihex bash

FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]
RUN apt-get -y update \
    && apt-get -y upgrade \
    && apt-get -y install \
    ros-noetic-ros-controllers

RUN echo "export DISPLAY=host.docker.internal:0.0" >> ~/.bashrc \
    && echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

RUN mkdir -p ~/catkin_ws/src
WORKDIR /root/catkin_ws/
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

# Commands for adding package dependencies
WORKDIR /root/catkin_ws/src/
RUN catkin_create_pkg millihex_robot std_msgs rospy
ADD . /root/catkin_ws/src/millihex_robot/

# # Commands for adding folders
# WORKDIR /root/catkin_ws/src/millihex_robot
# RUN mkdir config \
#     && mkdir launch \
#     && mkdir robot