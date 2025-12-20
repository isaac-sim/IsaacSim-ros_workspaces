#!/bin/bash

# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Display help function
function display_help {
  echo "Usage: $0 [options]"
  echo ""
  echo "Build the ROS2 workspace using Docker."
  echo ""
  echo "Options:"
  echo "  -d DISTRO     Specify ROS2 distro (humble or jazzy)"
  echo "  -v VERSION    Specify Ubuntu version (22.04 or 24.04)"
  echo "  -h            Display this help message"
  echo ""
  echo "Supported combinations:"
  echo "  - humble: Ubuntu 22.04"
  echo "  - jazzy:  Ubuntu 22.04 or 24.04"
  echo ""
  echo "If no options are specified, the script will detect your current Ubuntu version"
  echo "and select: Ubuntu 22.04 -> Humble, Ubuntu 24.04 -> Jazzy"
}

# Add command-line arguments
UBUNTU_VERSION=""
ROS_DISTRO=""
while getopts "v:d:h" opt; do
  case $opt in
    v) UBUNTU_VERSION=$OPTARG ;;
    d) ROS_DISTRO=$OPTARG ;;
    h) display_help; exit 0 ;;
    \?) echo "Invalid option -$OPTARG" >&2; display_help; exit 1 ;;
  esac
done

# Update git submodules in the repo
echo "Updating git submodules..."
git submodule update --init --recursive

# Detect Ubuntu version if not specified through flag
if [ -z "$UBUNTU_VERSION" ]; then
    echo "No Ubuntu version specified, detecting from system..."
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        UBUNTU_VERSION=${VERSION_ID}
        echo "Detected Ubuntu version: $UBUNTU_VERSION"
    else
        echo "Cannot detect Ubuntu version"
        exit 1
    fi
fi

# Set default ROS distro based on Ubuntu version if not specified
if [ -z "$ROS_DISTRO" ]; then
    if [ "$UBUNTU_VERSION" = "22.04" ]; then
        ROS_DISTRO="humble"
    elif [ "$UBUNTU_VERSION" = "24.04" ]; then
        ROS_DISTRO="jazzy"
    else
        echo "Unsupported Ubuntu version: $UBUNTU_VERSION"
        echo "Supported versions are: 22.04, 24.04"
        exit 1
    fi
    echo "No ROS distro specified, defaulting to $ROS_DISTRO based on Ubuntu version"
fi

# Validate the combination of Ubuntu version and ROS distro
if [ "$ROS_DISTRO" = "humble" ] && [ "$UBUNTU_VERSION" != "22.04" ]; then
    echo "Error: ROS Humble is only supported on Ubuntu 22.04"
    exit 1
fi

if [ "$ROS_DISTRO" = "jazzy" ] && [ "$UBUNTU_VERSION" != "22.04" ] && [ "$UBUNTU_VERSION" != "24.04" ]; then
    echo "Error: ROS Jazzy is only supported on Ubuntu 22.04 or 24.04"
    exit 1
fi

# Select the appropriate Docker file
if [ "$ROS_DISTRO" = "humble" ]; then
    DOCKERFILE="dockerfiles/ubuntu_22_humble_python_312_minimal.dockerfile"
    echo "Using Ubuntu 22.04 with ROS Humble"
elif [ "$ROS_DISTRO" = "jazzy" ]; then
    if [ "$UBUNTU_VERSION" = "22.04" ]; then
        DOCKERFILE="dockerfiles/ubuntu_22_jazzy_python_312_minimal.dockerfile"
        echo "Using Ubuntu 22.04 with ROS Jazzy"
    elif [ "$UBUNTU_VERSION" = "24.04" ]; then
        DOCKERFILE="dockerfiles/ubuntu_24_jazzy_python_312_minimal.dockerfile" 
        echo "Using Ubuntu 24.04 with ROS Jazzy"
    fi
else
    echo "Unsupported ROS distro: $ROS_DISTRO"
    echo "Supported distros are: humble, jazzy"
    exit 1
fi

# Build the Docker image
docker build . --network=host -f $DOCKERFILE -t isaac_sim_ros:ubuntu_${UBUNTU_VERSION%.*}_${ROS_DISTRO}

# Prepare the target directory
rm -rf build_ws/${ROS_DISTRO}
mkdir -p build_ws/${ROS_DISTRO}

pushd build_ws/${ROS_DISTRO}

# Extract files from Docker container
docker cp $(docker create --rm isaac_sim_ros:ubuntu_${UBUNTU_VERSION%.*}_${ROS_DISTRO}):/workspace/${ROS_DISTRO}_ws ${ROS_DISTRO}_ws

docker cp $(docker create --rm isaac_sim_ros:ubuntu_${UBUNTU_VERSION%.*}_${ROS_DISTRO}):/workspace/build_ws isaac_sim_ros_ws

popd

echo "Build complete for $ROS_DISTRO on Ubuntu $UBUNTU_VERSION" 