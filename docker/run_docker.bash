# Run docker container
#
# To get a specific ROS2 distro, specify the ROS_DISTRO environment variable as follows:
#   ROS_DISTRO=humble ./run_docker.bash
#
# To run a specific command, you can enter additional arguments:
#   ./run_docker.bash src/pyrobosim/test/run_tests.bash
#
# To run without display for use in continuous integration, use the `ci_mode` argument:
#   ./run_docker.bash src/pyrobosim/test/run_tests.bash ci_mode

if [ "$ROS_DISTRO" == "" ]
then
    echo "ROS Distro not specified or found in the ROS_DISTRO environment variable, using Foxy"
    ROS_DISTRO=foxy
fi

IMAGE_NAME=pyrobosim_$ROS_DISTRO
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# If no command is specified, just open a Bash terminal.
CMD="${1:-bash}"

# If running as part of CI, do not use displays and interactive mode.
if [ "$2" == "ci_mode" ]
then 
  DISPLAY_ARGS=""
else
  source $SCRIPT_DIR/setup_xauth.bash
  DISPLAY_ARGS="
    -it --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/dev/input:/dev/input" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
"
fi

# Setup other Docker arguments
NETWORK_ARGS="--ipc=host --net=host"
PYROBOSIM_WS="/pyrobosim_ws/src/pyrobosim"
VOLUMES="
  --volume=$SCRIPT_DIR/../pyrobosim:$PYROBOSIM_WS/pyrobosim:rw \
  --volume=$SCRIPT_DIR/../pyrobosim_msgs:$PYROBOSIM_WS/pyrobosim_msgs:rw \
  --volume=$SCRIPT_DIR/../pyrobosim_ros:$PYROBOSIM_WS/pyrobosim_ros:rw \
  --volume=$SCRIPT_DIR/../setup:$PYROBOSIM_WS/setup:rw \
  --volume=$SCRIPT_DIR/../test:$PYROBOSIM_WS/test:rw \
  --volume=$SCRIPT_DIR/tmp/build:/pyrobosim_ws/build:rw \
  --volume=$SCRIPT_DIR/tmp/install:/pyrobosim_ws/install:rw \
  --volume=$SCRIPT_DIR/tmp/log:/pyrobosim_ws/install/log:rw \
"
# Finally, run the command in Docker
docker run --rm $NETWORK_ARGS $DISPLAY_ARGS $VOLUMES \
    $IMAGE_NAME $CMD
