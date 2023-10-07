#!/bin/bash

# Install yq if it's not already installed
if ! command -v yq &> /dev/null
then
    echo "yq could not be found, goto https://github.com/mikefarah/yq#install"

    if [[ "$OSTYPE" == "darwin"* ]]; then
        # Mac OSX
        echo probably run "brew install yq"
    fi

    exit 1
fi

# Parse the values from the config.yaml file
USERNAME=$(yq e '.USERNAME' config.yaml)
IMAGE_NAME=$(yq e '.IMAGE_NAME' config.yaml)
TAG=$(yq e '.TAG' config.yaml)

FULL_IMAGE_NAME="$USERNAME/$IMAGE_NAME:$TAG"

# make sure buildx is installed, and alias "docker build" to "docker buildx"
docker buildx install 

# build image for multiple platforms, push latest to docker.io
docker build \
    -t $FULL_IMAGE_NAME \
    -f dockerfiles/Dockerfile . \
    --platform="linux/arm64,linux/amd64" \
    --push

# and then load image into local registry
# you should see the image with "docker image ls" 
echo loading $FULL_IMAGE_NAME into docker image registry
docker build \
    -t $FULL_IMAGE_NAME \
    -f dockerfiles/Dockerfile . \
    --load

# Get the list of Docker images and search for the specified image
IMAGE_FOUND=$(docker image ls | grep "$USERNAME/$IMAGE_NAME")

# Check whether the specified image was found
if [[ -n $IMAGE_FOUND ]]; then
    echo "An image $USERNAME/$IMAGE_NAME was found in image registry."
else
    echo "An image $USERNAME/$IMAGE_NAME was not found, there is an issue :("
    exit 1
fi

# # Start a Docker container and get Python and PyTorch versions
PYTHON_VERSION=$(docker run --rm -ti $FULL_IMAGE_NAME python -c 'import sys; print(".".join(map(str, sys.version_info[:3])), end="")')
TORCH_VERSION=$(docker run --rm -ti $FULL_IMAGE_NAME python -c 'import torch; print(torch.__version__, end="")')

# Construct image tag with python and torch version
NEW_TAG="${TORCH_VERSION//+/_}-python${PYTHON_VERSION}-devell"
WITH_VERSIONS_FULL_IMAGE_NAME="$USERNAME/$IMAGE_NAME:$NEW_TAG"

# build image for multiple platforms, push latest to docker.io
docker build \
    -t $WITH_VERSIONS_FULL_IMAGE_NAME \
    -f dockerfiles/Dockerfile . \
    --platform="linux/arm64,linux/amd64" \
    --push

# and then load image into local registry
# you should see the image with "docker image ls"
docker build \
    -t $WITH_VERSIONS_FULL_IMAGE_NAME \
    -f dockerfiles/Dockerfile . \
    --load


