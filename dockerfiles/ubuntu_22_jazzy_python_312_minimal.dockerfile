ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=jazzy_ws
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release


# Upgrade installed packages
RUN apt update && apt upgrade -y && apt clean

# Install Python3.12
RUN apt update && \
    apt install --no-install-recommends -y build-essential software-properties-common && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt install --no-install-recommends -y python3.12 python3.12-dev python3.12-venv

# Setting up locale stuff
RUN apt update && apt install locales

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Set default Python3 to Python3.12
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.12 1

# Pip install stuff
RUN curl -s https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3.12 get-pip.py --force-reinstall && \
    rm get-pip.py

RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc && apt-key add ros.asc
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Additional dependencies needed for rosidl_generator_c
RUN apt update && apt install -y \
    pkg-config \
    python3-yaml \
    cmake-extras

# Install Boost libraries needed for OMPL
RUN apt update && apt install -y \
    libboost-all-dev \
    libboost-dev \
    libboost-filesystem-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-serialization-dev \
    libboost-date-time-dev \
    libboost-regex-dev \
    libboost-python-dev \
    libfmt-dev

# Install dependencies for geometric_shapes and other packages
RUN apt update && apt install -y \
    libqhull-dev \
    libassimp-dev \
    liboctomap-dev \
    libconsole-bridge-dev \
    libfcl-dev

# Install Eigen3 needed for OMPL and MoveIt
RUN apt update && apt install -y \
    libeigen3-dev

# Install X11 and graphics dependencies needed for OGRE (RViz)
RUN apt update && apt install -y \
    libx11-dev \
    libxaw7-dev \
    libxrandr-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglew-dev \
    libgles2-mesa-dev \
    libopengl-dev \
    libfreetype-dev \
    libfreetype6-dev \
    libfontconfig1-dev \
    libfmt-dev

# Install Qt5 and additional dependencies for RViz
RUN apt update && apt install -y \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libqt5core5a \
    libqt5gui5 \
    libqt5opengl5 \
    libqt5widgets5 \
    libxcursor-dev \
    libxinerama-dev \
    libxi-dev \
    libyaml-cpp-dev \
    libassimp-dev \
    libzzip-dev \
    freeglut3-dev \
    libogre-1.9-dev \
    libpng-dev \
    libjpeg-dev \
    python3-pyqt5.qtwebengine

RUN pip3 install setuptools==70.0.0

RUN apt update && apt install -y \
  python3-pip \
  python3-pytest-cov \
  python3-rosinstall-generator \
  ros-dev-tools \
  libbullet-dev \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev \
  libacl1-dev \
  python3-empy \
  libpython3-dev \
  liblttng-ust-dev

# Install the correct version of empy that is compatible with ROS 2 jazzy
# Uninstall any existing empy first, then install version 3.3.4 specifically
RUN python3.12 -m pip uninstall -y em empy || true
RUN python3.12 -m pip install empy==3.3.4

RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  lark

RUN python3.12 -m pip uninstall numpy -y
RUN python3.12 -m pip install --upgrade pip
RUN python3.12 -m pip install numpy pybind11 PyYAML

# Create symlinks for Python3.12 headers where CMake can find them
RUN ln -sf /usr/include/python3.12 /usr/include/python3

# Fix paths for pybind11
RUN python3.12 -m pip install "pybind11[global]"

RUN mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT} && \
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} rosidl_runtime_c rcutils rcl rmw tf2 tf2_msgs common_interfaces geometry_msgs nav_msgs std_msgs rosgraph_msgs sensor_msgs vision_msgs rclpy ros2topic ros2pkg ros2doctor ros2run ros2node ros_environment ackermann_msgs example_interfaces > ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall

# Patch rclpy to ensure it builds with Python 3.12 - find the correct path first
RUN find /workspace/${ROS_ROOT}/src -name rclpy -type d | xargs -I{} /bin/bash -c 'if [ -f {}/CMakeLists.txt ]; then \
    echo "Patching {}/CMakeLists.txt"; \
    sed -i "s/include_directories(\${PYTHON_INCLUDE_DIRS})/include_directories(\/usr\/include\/python3.12)/" {}/CMakeLists.txt; \
    sed -i "s/\${PYTHON_LIBRARY}/python3.12/" {}/CMakeLists.txt; \
    fi'

RUN rosdep init && rosdep update

# Make sure PYTHONPATH includes the correct site-packages
ENV PYTHONPATH=/usr/local/lib/python3.12/dist-packages

# Use logging to help debug build issues
RUN cd ${ROS_ROOT} && colcon build --cmake-args \
    "-DPython3_EXECUTABLE=/usr/bin/python3.12" \
    "-DPYTHON_EXECUTABLE=/usr/bin/python3.12" \
    "-DPYTHON_INCLUDE_DIR=/usr/include/python3.12" \
    "-DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.12.so" \
    --merge-install

# Need these to maintain compatibility on non 20.04 systems
RUN cp /usr/lib/x86_64-linux-gnu/libtinyxml2.so* /workspace/jazzy_ws/install/lib/ || true
RUN cp /usr/lib/x86_64-linux-gnu/libssl.so* /workspace/jazzy_ws/install/lib/ || true
RUN cp /usr/lib/x86_64-linux-gnu/libcrypto.so* /workspace/jazzy_ws/install/lib/ || true
RUN cp /usr/lib/x86_64-linux-gnu/liblttng-ust*.so* /workspace/jazzy_ws/install/lib/ || true

# Next, build the additional workspace 
RUN mkdir -p /workspace/build_ws/src


# Copy the source files only - don't copy any build artifacts
COPY jazzy_ws/src /workspace/build_ws/src

# Removing MoveIt packages from the internal ROS Python 3.12 library build as it uses standard interfaces already built above.
# This is to ensure that the internal build is as minimal as possible. 
# For the user facing MoveIt interface workflow, this package should be built with the rest of the workspace uisng the external ROS installation.
RUN rm -rf /workspace/build_ws/src/moveit

# Make sure we're in the right directory
WORKDIR /workspace

# Set up environment variables for Python 3.12
ENV PYTHONPATH=/usr/local/lib/python3.12/dist-packages
ENV PYTHON_EXECUTABLE=/usr/bin/python3.12
ENV Python3_EXECUTABLE=/usr/bin/python3.12
ENV PYTHON_INCLUDE_DIR=/usr/include/python3.12
ENV PYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.12.so

# Build the workspace with Python 3.12
RUN /bin/bash -c "source ${ROS_ROOT}/install/setup.sh && cd build_ws && colcon build --cmake-args \
    '-DPython3_EXECUTABLE=/usr/bin/python3.12' \
    '-DPYTHON_EXECUTABLE=/usr/bin/python3.12' \
    '-DPYTHON_INCLUDE_DIR=/usr/include/python3.12' \
    '-DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.12.so'"