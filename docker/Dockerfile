ARG ROS_DISTRO=humble

# pyrobosim Dockerfile for Ubuntu / ROS
FROM osrf/ros:${ROS_DISTRO}-desktop
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
COPY setup /pyrobosim_ws/src/setup
# RUN cd /pyrobosim_ws/src/setup && ./setup_pddlstream.bash

# Install pyrobosim and testing dependencies
WORKDIR /pyrobosim_ws
COPY . /pyrobosim_ws/
RUN pip3 install ./pyrobosim
RUN pip3 install lark pytest pytest-dependency pytest-html

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Setup an entrypoint and working folder
CMD ["/usr/bin/bash"]
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
