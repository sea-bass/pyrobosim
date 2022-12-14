# Build docker image
if [ "$ROS_DISTRO" == "" ]
then
    echo "ROS Distro not specified or found in the ROS_DISTRO environment variable, using Humble"
    ROS_DISTRO=humble
fi

IMAGE_NAME=pyrobosim_$ROS_DISTRO
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

rm -rf $SCRIPT_DIR/tmp

pushd $SCRIPT_DIR/..
docker build -f $SCRIPT_DIR/Dockerfile \
             --build-arg ROS_DISTRO=$ROS_DISTRO \
             -t $IMAGE_NAME .
popd
