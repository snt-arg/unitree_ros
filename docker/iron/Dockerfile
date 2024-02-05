FROM ros:iron-ros-base

# - ssh keys to get the s_graphs repo (only temporary)
ARG ssh_prv_key
ARG ssh_pub_key

# - Disable the interaction if required
ENV DEBIAN_FRONTEND=noninteractive


RUN apt-get update && apt-get install -y --no-install-recommends libtool \
    wget nano vim build-essential git python3 python3-pip python3-vcstool git libboost-all-dev

# Authorize SSH Host (only temporary)
RUN mkdir -p /root/.ssh && \
    chmod 0700 /root/.ssh && \
    ssh-keyscan github.com > /root/.ssh/known_hosts

RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    echo "$ssh_pub_key" > /root/.ssh/id_rsa.pub && \
    chmod 600 /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa.pub

WORKDIR /root/unitree_ws/src
RUN git clone --recurse-submodules git@github.com:snt-arg/unitree_ros.git
WORKDIR /root/unitree_ws/

RUN . /opt/ros/iron/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN rm -rf /root/.ssh

WORKDIR /root/unitree_ws/src/unitree_ros
