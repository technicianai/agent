FROM ubuntu:focal
ENV UBUNTU_VERSION=focal
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ROS 1
RUN apt-get update && apt-get install -y curl gnupg2
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VERSION main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt update && apt install -y ros-noetic-ros-base

# ROS 2
RUN apt update && sudo apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt install -y ros-foxy-ros-base python3-argcomplete ros-dev-tools

# Non-ROS dependencies
RUN apt-get update && apt-get install -y \
    nlohmann-json3-dev \
    openssl \
    openssh-client \
    uuid-dev \
    wget
RUN mkdir downloads
RUN cd downloads && \
    git clone https://github.com/eclipse/paho.mqtt.c.git && \
    cd paho.mqtt.c && \
    mkdir build && \
    cd build && \
    cmake .. -DPAHO_WITH_SSL=TRUE && \
    make && \
    make install && \
    ldconfig
RUN cd downloads && \
    git clone https://github.com/eclipse/paho.mqtt.cpp && \
    cd paho.mqtt.cpp && \
    cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON && \
    cmake --build build/ --target install && \
    ldconfig

# ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-foxy-cv-bridge \
    ros-foxy-vision-opencv \
    ros-foxy-rosbag2* \
    ros-foxy-can-msgs \
    ros-foxy-nmea-msgs \
    ros-noetic-rosbag \
    ros-noetic-rosbridge-server \
    ros-noetic-topic-tools \
    ros-noetic-rosbag-snapshot
RUN cd downloads && \
    git clone https://github.com/IntelRealSense/realsense-ros.git && \
    cd realsense-ros && \
    git checkout ros1-legacy    
RUN cd downloads && \
    git clone https://github.com/stereolabs/zed-ros-interfaces.git

# Create workspace
WORKDIR /woeden_agent
COPY certs/ /woeden_agent/certs/
COPY scripts/ /woeden_agent/scripts/
COPY src/ /woeden_agent/
RUN cp -a /downloads/realsense-ros/realsense2_camera/msg/. /woeden_agent/interfaces/msg
RUN cp -a /downloads/zed-ros-interfaces/msg/. /woeden_agent/interfaces/msg

RUN wget https://bootstrap.pypa.io/get-pip.py
RUN python3 get-pip.py && python3 -m pip install stream-zip rosbags matplotlib
RUN . /opt/ros/noetic/setup.bash && python3 -m pip install rosbag-merge

RUN . /opt/ros/foxy/setup.bash && colcon build --packages-skip ros1_bridge
RUN . /opt/ros/noetic/setup.bash && \
    . /opt/ros/foxy/setup.bash && \
    . /woeden_agent/install/setup.bash && \
    MAKEFLAGS="-j10 -l1" colcon build \
        --symlink-install \
        --packages-select ros1_bridge \
        --cmake-force-configure \
        --executor sequential

COPY docker/entrypoint.sh /woeden_agent/

ENV HOME /
ENV MQTT_HOST ssl://mqtt.woeden.com

RUN rm -rf /woeden_agent/src/ /downloads/
RUN mkdir /.ros
RUN chmod -R 777 /.ros

CMD ["./entrypoint.sh"]
