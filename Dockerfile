# syntax=docker/dockerfile:1

# Build Docker Image from scratch:
#   DOCKER_BUILDKIT=1 docker build -t cmatchan/millihexapod:<tag> .
#
# Pull Docker Image from Dockerhub:
#   docker pull cmatchan/millihexapod:<tag>
#
# Run Docker Image in a new Container:
#   docker run --name millihex -it cmatchan/millihexapod:<tag> bash

# Install ROS Noetic and package dependenceis
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]
RUN apt-get -y update \
    && apt-get -y upgrade \
    && apt-get -y install \
    ros-noetic-ros-controllers \
    git

# Set DISPLAY and OpenGL environment variables in .bashrc
RUN echo "export DISPLAY=host.docker.internal:0.0" >> ~/.bashrc \
    && echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

# Build catkin_ws and source workspace
RUN mkdir -p ~/catkin_ws/src
WORKDIR /root/catkin_ws/
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

# Create working directory for robot development
WORKDIR /root/catkin_ws/src/
RUN git clone https://github.com/cmatchan/millihexapod.git
WORKDIR /root/catkin_ws