#!/bin/bash

# magic line that allows graphics
xhost +local:root


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change the working directory to the script's directory
cd "$SCRIPT_DIR/.."

SF_NAME="sources"

IMAGE_NAME="fabioscap/fodom"
CONTAINER_NAME="fodom"


function create_and_run {
    echo "Creating and starting a new container $CONTAINER_NAME..."
    docker run -it \
                --privileged \
                --gpus all \
                --runtime=nvidia \
                -e NVIDIA_DRIVER_CAPABILITIES=all \
                -e DISPLAY=$DISPLAY \
                -v $PWD:/root/$SF_NAME \
                -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
                -v ~/.ssh:/root/.ssh \
                --device=/dev/dri:/dev/dri \
                --device /dev/nvidia0:/dev/nvidia0 \
                --device /dev/nvidiactl:/dev/nvidiactl \
                --device /dev/nvidia-uvm:/dev/nvidia-uvm \
                --device /dev/nvidia-uvm-tools:/dev/nvidia-uvm-tools \
                --name $CONTAINER_NAME $IMAGE_NAME bash
}

RESET=0
REBUILD=0
if [ "$1" = "reset" ]; then
  echo "RESET"
  RESET=1
elif [ "$1" = "rebuild" ]; then
  echo "REBUILD"
  RESET=1
  REBUILD=1
fi


RUNNING=0 # there exists a running container
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
  echo "Found running container $CONTAINER_NAME"
  RUNNING=1
fi

STOPPED=0 # there exists a stopped container
if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
  echo "Found stopped container $CONTAINER_NAME"
  STOPPED=1
fi


if (( $RESET )); then
  if (( $RUNNING )); then
    echo "Stopping"
    docker stop $CONTAINER_NAME
    echo "Removing"
    docker rm $CONTAINER_NAME
  elif (( $STOPPED )); then
    echo "Removing container $CONTAINER_NAME..."
    docker rm $CONTAINER_NAME
  fi
  if (( $REBUILD )); then
  echo "Rebuilding image $IMAGE_NAME"
  docker/build.sh
  fi
  create_and_run

elif (( $RUNNING )); then
  echo "Attaching to running container $CONTAINER_NAME"
  docker exec -it $CONTAINER_NAME bash
elif (( $STOPPED )); then
  echo "Starting container"
  docker start $CONTAINER_NAME 
  docker exec -it $CONTAINER_NAME bash 
else
  create_and_run
fi

cd -
