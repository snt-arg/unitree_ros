FROM ros:foxy-ros-base

# - Disable the interaction if required
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends libtool \
    wget nano vim build-essential git python3 python3-pip python3-vcstool git \
    iputils-ping libboost-all-dev

WORKDIR /root/unitree_ws/src
RUN git clone -b foxy --recurse-submodules https://github.com/snt-arg/unitree_ros.git
WORKDIR /root/unitree_ws/

RUN . /opt/ros/foxy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

WORKDIR /root/unitree_ws/src/unitree_ros
