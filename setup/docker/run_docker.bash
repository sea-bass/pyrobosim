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

DISPLAY_ARGS="
--gpus all \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
"

# If running as part of CI, disable displays and interactive mode.
INTERACT_FLAGS=-it
if [ "$2" == "ci_mode" ]
then 
  INTERACT_FLAGS=""
  DISPLAY_ARGS=""
fi

# Finally, run the command in Docker
docker run $INTERACT_FLAGS --net=host $DISPLAY_ARGS \
    --volume=$MAIN_DIR:/pyrobosim_ws/src/pyrobosim \
    $IMAGE_NAME $CMD
