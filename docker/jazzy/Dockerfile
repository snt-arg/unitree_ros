FROM ros:jazzy-ros-base

# -----------------------------------------------------------------------------
# Environment
# -----------------------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# -----------------------------------------------------------------------------
# Install system dependencies
# -----------------------------------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        git \
        iputils-ping \
        libboost-all-dev \
        libtool \
        nano \
        python3 \
        python3-pip \
        python3-vcstool \
        vim \
        wget && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Create workspace
# -----------------------------------------------------------------------------
WORKDIR /root/unitree_ws/src/unitree_ros

# Copy only dependency manifests first to maximize Docker cache reuse.
COPY package.xml .
COPY CMakeLists.txt .

# -----------------------------------------------------------------------------
# Install ROS dependencies
# -----------------------------------------------------------------------------
WORKDIR /root/unitree_ws

RUN apt-get update && \
    bash -c "\
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        --reinstall \
        -r \
        -y \
"

# -----------------------------------------------------------------------------
# Copy package sources
# -----------------------------------------------------------------------------
WORKDIR /root/unitree_ws/src/unitree_ros

COPY CHANGELOG.rst .
COPY LICENSE .

COPY include/ include/
COPY src/ src/
COPY msg/ msg/
COPY launch/ launch/
COPY config/ config/
COPY utils/ utils/
COPY unitree_hardware_interface.xml .

# -----------------------------------------------------------------------------
# Build workspace
# -----------------------------------------------------------------------------
WORKDIR /root/unitree_ws

RUN bash -c "\
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
"

# -----------------------------------------------------------------------------
# Automatically source ROS + workspace
# -----------------------------------------------------------------------------
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo 'source /root/unitree_ws/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
