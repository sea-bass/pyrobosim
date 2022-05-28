# Run docker container
#
# To get display, first run the following command in your Terminal: 
#   xhost + local:docker

ROS_DISTRO=foxy
IMAGE_NAME=pyrobosim_$ROS_DISTRO

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
MAIN_DIR=$SCRIPT_DIR/../..

# If no command is specified, just open a terminal
CMD=${1:-bash}

docker run -it --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1"\
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=$MAIN_DIR:/pyrobosim_ws/src/pyrobosim \
    $IMAGE_NAME $CMD
