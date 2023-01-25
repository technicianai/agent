FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    openssl \
    openssh-client \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-rosbridge-suite \
    ros-humble-vision-opencv \
    uuid-dev

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

WORKDIR /woeden_agent

COPY bag_utils/ /woeden_agent/bag_utils/
RUN python3 bag_utils/get-pip.py && python3 -m pip install stream-zip imageio pandas rosbags

COPY src/ /woeden_agent/
RUN . /opt/ros/humble/setup.bash && colcon build

COPY certs/ /woeden_agent/certs/
RUN chmod 400 /woeden_agent/certs/gateway.pem

COPY ros-entrypoint.sh /woeden_agent/

ENV HOME /
ENV HOST ssl://mqtt.woeden.com
ENV FASTRTPS_DEFAULT_PROFILES_FILE /super_client_configuration_file.xml

RUN rm -rf /woeden_agent/src/

CMD ["./ros-entrypoint.sh"]
