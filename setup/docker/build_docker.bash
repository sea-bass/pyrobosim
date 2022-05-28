# Build docker image
ROS_DISTRO=foxy
IMAGE_NAME=pyrobosim_$ROS_DISTRO
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

pushd $SCRIPT_DIR
docker build -f $SCRIPT_DIR/Dockerfile.$ROS_DISTRO -t $IMAGE_NAME .
popd