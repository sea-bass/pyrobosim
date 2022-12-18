ARG ROS_DISTRO=humble

# pyrobosim Dockerfile for Ubuntu / ROS
FROM osrf/ros:${ROS_DISTRO}-desktop
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=${ROS_DISTRO}

# Install dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y 
RUN apt-get install -y \ 
    apt-utils python3-pip python3-tk

# Create a colcon workspace
RUN mkdir -p /pyrobosim_ws/src/pyrobosim

# Install dependencies
COPY setup /pyrobosim_ws/src/pyrobosim/setup
RUN cd /pyrobosim_ws/src/pyrobosim/setup && ./setup_pddlstream.bash

# Install pyrobosim and testing dependencies
COPY pyrobosim /temp/pyrobosim
RUN cd /temp/pyrobosim && pip3 install .
RUN pip3 install lark pytest pytest-dependency pytest-html

# Setup an entrypoint and working folder
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /pyrobosim_ws
