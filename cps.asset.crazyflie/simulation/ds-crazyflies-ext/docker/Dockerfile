# syntax=docker/dockerfile:1
FROM osrf/ros:jazzy-desktop-full AS base
SHELL ["/bin/bash", "-c"]

# Value injected by prepare.sh
ENV ROS_DISTRO=jazzy

RUN apt-get update

# Preconfigure keyboard + locale before installing webots
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      locales \
      keyboard-configuration && \
    locale-gen en_US.UTF-8

# Create the user and add sudo support
RUN apt-get update \
    && apt-get install -y sudo udev curl usbutils nano graphviz git

# Install some basic packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip gdb dos2unix wget

RUN apt-get install mesa-utils -y 

# Install ros dependencies
RUN apt-get install -y ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Install webots
RUN mkdir -p /etc/apt/keyrings
WORKDIR /etc/apt/keyrings
RUN wget -q https://cyberbotics.com/Cyberbotics.asc

RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
RUN apt-get update

RUN apt-get install webots -y

ENV WEBOTS_HOME=/usr/local/webots

RUN mkdir -p /ds && chmod 777 /ds
ENV USER_UID=1000
ENV USER_GID=$USER_UID
RUN useradd -ms /bin/bash devuser && \
    echo "devuser ALL=(root) NOPASSWD:ALL" >> /etc/sudoers.d/devuser && \
    chmod 0440 /etc/sudoers.d/devuser
#USER devuser

RUN git config --global --add safe.directory /ds/ds-crazyflies

# Install ds-crazyflies
WORKDIR /ds
COPY ./src /ds/ds-crazyflies/src
COPY build.sh /ds/ds-crazyflies/

WORKDIR /ds/ds-crazyflies
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
    dos2unix build.sh &&\
    bash build.sh

# Install webotsworld
WORKDIR /ds
RUN git clone https://github.com/DynamicSwarms/crazywebotsworld.git

# Copy Webots world file
# Now bind-mounted instead for better development pipeline
#COPY ./crazyflie.wbt /ds/crazywebotsworld/worlds/crazyflie.wbt
  
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /ds/ds-crazyflies/install/setup.bash" >> ~/.bashrc
