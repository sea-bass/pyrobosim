# Dockerfile for pyrobosim builds with and without ROS
ARG ROS_DISTRO=humble

##################################
#
# base pyrobosim build for Ubuntu
#
##################################
FROM ubuntu:latest as pyrobosim

# Install dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update &&\
    apt-get install -y \
        apt-utils \
        python3 \
        python3-pip \
        python3-tk \
        git \
        cmake \
        ffmpeg \
        libsm6 \
        libxext6

RUN mkdir -p /opt/pyrobosim/test
RUN mkdir -p /opt/pyrobosim/setup
WORKDIR /opt/pyrobosim/

# Install PDDLStream
COPY setup/setup_pddlstream.bash setup/
RUN setup/setup_pddlstream.bash

# Install pip dependencies for testing
COPY test/python_test_requirements.txt test/
RUN pip3 install --break-system-packages -r test/python_test_requirements.txt

# Copy the rest of the source directory and install pyrobosim
COPY . /opt/pyrobosim/
RUN pip3 install --break-system-packages -e pyrobosim

# Set the default startup command
CMD /bin/bash

##################################
#
# pyrobosim build for Ubuntu / ROS
#
##################################
FROM ros:${ROS_DISTRO}-ros-base as pyrobosim_ros
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    apt-utils python3-pip python3-tk

# Create a colcon workspace
RUN mkdir -p /pyrobosim_ws/src/pyrobosim

# Install dependencies
COPY setup/setup_pddlstream.bash /pyrobosim_ws/src/setup/
WORKDIR /pyrobosim_ws/src/setup
RUN ./setup_pddlstream.bash

# Install pyrobosim and testing dependencies
WORKDIR /pyrobosim_ws/src
COPY . /pyrobosim_ws/src/
RUN python3 -m pip install --upgrade pip
RUN pip3 install ./pyrobosim
RUN pip3 install -r test/python_test_requirements.txt

# Build the workspace
WORKDIR /pyrobosim_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Setup an entrypoint and working folder
COPY setup /pyrobosim_ws/src/setup
CMD /bin/bash
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
