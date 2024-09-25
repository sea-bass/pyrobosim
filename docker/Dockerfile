# Dockerfile for pyrobosim builds with and without ROS
ARG ROS_DISTRO=humble

##################################
#
# base pyrobosim build for Ubuntu
#
##################################
FROM ubuntu:latest AS pyrobosim

# Install dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update &&\
    apt-get install -y \
        apt-utils \
        libegl1 \
        libgl1-mesa-dev \
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
COPY setup/configure_pddlstream.bash setup/
RUN setup/configure_pddlstream.bash

# Install pip dependencies for testing
ENV PIP_BREAK_SYSTEM_PACKAGES=1
COPY test/python_test_requirements.txt test/
RUN pip3 install -r test/python_test_requirements.txt

# Copy the rest of the source directory and install pyrobosim
COPY . /opt/pyrobosim/
RUN pip3 install -e pyrobosim

# Set the default startup command
CMD /bin/bash

##################################
#
# pyrobosim build for Ubuntu / ROS
#
##################################
FROM ros:${ROS_DISTRO}-ros-base AS pyrobosim_ros
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    apt-utils python3-pip python3-tk \
    libegl1 libgl1-mesa-dev libglu1-mesa-dev '^libxcb.*-dev' libx11-xcb-dev \
    libxi-dev libxkbcommon-dev libxkbcommon-x11-dev libxrender-dev

# Create a colcon workspace
RUN mkdir -p /pyrobosim_ws/src/pyrobosim

# Install dependencies
COPY setup/configure_pddlstream.bash /pyrobosim_ws/src/setup/
WORKDIR /pyrobosim_ws/src/setup
RUN ./configure_pddlstream.bash

COPY . /pyrobosim_ws/src/

# Install pyrobosim and testing dependencies
ENV PIP_BREAK_SYSTEM_PACKAGES=1
WORKDIR /pyrobosim_ws/src
RUN pip3 install -e ./pyrobosim && \
    pip3 install -r test/python_test_requirements.txt

# Build the ROS workspace
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
