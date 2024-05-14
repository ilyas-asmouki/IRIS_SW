# ====================================================================================
#     Xplore Rover Challenge IRIS 2024    
#     Ubuntu + ROS Humble Raspberry Pi image with up-to-date workspace                      
# ====================================================================================

FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /root
                           
RUN echo "\nStarting the Raspberry Pi 3B+ ROS2 Humble install script\n" \
    "This script is for a Ubuntu 22.04 server (no GUI) version\n" \
    "This script should take less than 10 minutes to finish\n"

# Configure locales
RUN apt-get update && apt-get install -y locales && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Update system and install core tools and dependencies in one layer
RUN apt-get update && apt-get install -y \
    vim \
    htop \
    neofetch \
    git \
    build-essential \
    python3-gpiozero \
    software-properties-common \
    curl \
    libserial-dev \
    cmake \
    && add-apt-repository universe -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y ros-humble-ros-base ros-dev-tools \
    && apt install \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Sourcing ROS & adding it to bashrc
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"

# Import code from github and build it
RUN echo "Edit this when git repo is updated, last update 05.05.2004\n" && \
    git clone https://github.com/ilyas-asmouki/Moukafa7a.git && \
    mv Moukafa7a ros2_ws

WORKDIR /root/ros2_ws/src/serial
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install

WORKDIR /root/ros2_ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  colcon build --packages-skip camera_node"

WORKDIR /root

# Finish installation message
RUN echo "\nInstall + build script is done\n"