# syntax=docker/dockerfile:1
# ---------------------------------------------------------------------------
# unitree_ros devcontainer – parameterized by ROS distro
# ---------------------------------------------------------------------------
# Choose ROS 2 distro at build time:
#   foxy   (Ubuntu 20.04 / focal)
#   humble (Ubuntu 22.04 / jammy) – LTS
#   jazzy  (Ubuntu 24.04 / noble) – LTS
#   kilted (Ubuntu 24.04 / noble)
#
# Example builds:
#   docker build --build-arg ROS_DISTRO=humble -f .devcontainer/Dockerfile .
#   docker build --build-arg ROS_DISTRO=jazzy  -f .devcontainer/Dockerfile .
# ---------------------------------------------------------------------------
ARG ROS_DISTRO=kilted
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Re-declare for shell use
SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# System dependencies (union of docker/* Dockerfiles + dev tools)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        wget \
        curl \
        nano \
        vim \
        sudo \
        gnupg2 \
        lsb-release \
        ca-certificates \
        iputils-ping \
        net-tools \
        libtool \
        libboost-all-dev \
        python3 \
        python3-pip \
        python3-vcstool \
        python3-colcon-common-extensions \
        python3-rosdep \
        clang \
        clang-format \
        clangd \
        gdb \
        ccache \
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Create non-root user (matches VS Code devcontainer expectations)
# NOTE: Ubuntu version depends on ROS_DISTRO base image, user creation is
# distro-agnostic.
# ---------------------------------------------------------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} || true \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && usermod -aG dialout ${USERNAME} || true

# ---------------------------------------------------------------------------
# Install rosdep + initialize (idempotent)
# ---------------------------------------------------------------------------
RUN rosdep init 2>/dev/null || true \
    && rosdep update || true

# ---------------------------------------------------------------------------
# Auto-source ROS for all shells (both root and ros user)
# Bake distro path at build time so no runtime ENV expansion needed
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc \
    && echo 'if [ -f /workspaces/unitree_ros/install/setup.bash ]; then source /workspaces/unitree_ros/install/setup.bash; fi' >> /etc/bash.bashrc

USER ${USERNAME}
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo 'if [ -f /workspaces/unitree_ros/install/setup.bash ]; then source /workspaces/unitree_ros/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc \
    && echo 'if [ -f "$HOME/.bashrc" ]; then source "$HOME/.bashrc"; fi' >> /home/${USERNAME}/.bash_profile || true

WORKDIR /workspaces/unitree_ros

ENV DEBIAN_FRONTEND=dialog

LABEL org.opencontainers.image.source="https://github.com/snt-arg/unitree_ros"
LABEL devcontainer.rosdistro="${ROS_DISTRO}"

CMD ["/bin/bash"]
