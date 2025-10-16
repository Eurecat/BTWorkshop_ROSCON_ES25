#!/usr/bin/env bash
#
# Build script for ROSCON_ES25 BT Docker container
#
# Usage:
# - Default: ./build_container.sh
#

# export DOCKER_BUILDKIT=1
export ROS_DISTRO=jazzy

set -e

# --- BEGIN: Manage .env file ---
ENV_FILE="./.env" # Assuming .env is in the same directory as this script
# Create .env if it doesn't exist
if [ ! -f "$ENV_FILE" ]; then
    echo "Creating $ENV_FILE..."
    touch "$ENV_FILE"
fi

DEFAULT_USERNAME="btdev"
DEFAULT_ROS_DOMAIN_ID="1"
CURRENT_USER_ID=$(id -u)
CURRENT_GROUP_ID=$(id -g)

# Set USERNAME if not present, otherwise leave existing value
if ! grep -q -E "^USERNAME=" "$ENV_FILE"; then
    echo "USERNAME=$DEFAULT_USERNAME" >> "$ENV_FILE"
fi

# Set ROS_DOMAIN_ID if not present, otherwise leave existing value
if ! grep -q -E "^ROS_DOMAIN_ID=" "$ENV_FILE"; then
    echo "ROS_DOMAIN_ID=$DEFAULT_ROS_DOMAIN_ID" >> "$ENV_FILE"
fi

# Set or Update USER_ID with current host user ID
if grep -q -E "^USER_ID=" "$ENV_FILE"; then
    sed -i "s/^USER_ID=.*/USER_ID=$CURRENT_USER_ID/" "$ENV_FILE"
else
    echo "USER_ID=$CURRENT_USER_ID" >> "$ENV_FILE"
fi

# Set or Update GROUP_ID with current host group ID
if grep -q -E "^GROUP_ID=" "$ENV_FILE"; then
    sed -i "s/^GROUP_ID=.*/GROUP_ID=$CURRENT_GROUP_ID/" "$ENV_FILE"
else
    echo "GROUP_ID=$CURRENT_GROUP_ID" >> "$ENV_FILE"
fi
# --- END: Manage .env file ---

# Define directories
DEPS_DIR="./deps"

# Create deps directory if it doesn't exist
if [ ! -d $DEPS_DIR ]; then
    mkdir -p $DEPS_DIR
fi

# Check if --clean-rebuild is among the arguments
REBUILD=false
for arg in "$@"; do
    if [ "$arg" == "--clean-rebuild" ]; then
        REBUILD=true
    fi
done

# Import/update repository using VCS tools
echo "Importing repository using VCS..."
vcs import ${DEPS_DIR} < pyrobosim.repos
vcs pull ${DEPS_DIR}

# Build base image of the sim in its last release
cd deps/pyrobosim && docker compose build && cd -

BASE_IMAGE="pyrobosim_ros:${ROS_DISTRO}"
IMAGE_NAME="bt_roscon_es_25:${ROS_DISTRO}"

echo "Base image: ${BASE_IMAGE}"
echo "Output image: ${IMAGE_NAME}"


if $REBUILD; then
    echo "Rebuilding the application Docker image with no cache..."
    docker build --no-cache . --build-arg BASE_IMAGE="${BASE_IMAGE}" -t ${IMAGE_NAME} -f Dockerfile
else
    docker build . --build-arg BASE_IMAGE="${BASE_IMAGE}" -t ${IMAGE_NAME} -f Dockerfile
fi

# Set or Update BUILT_IMAGE 
if grep -q -E "^BUILT_IMAGE=" "$ENV_FILE"; then
    sed -i "s/^BUILT_IMAGE=.*/BUILT_IMAGE=$IMAGE_NAME/" "$ENV_FILE"
else
    echo "BUILT_IMAGE=$IMAGE_NAME" >> "$ENV_FILE"
fi

echo "Application Docker image ${IMAGE_NAME} built successfully!"
