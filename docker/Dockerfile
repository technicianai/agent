FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    openssl \
    openssh-client \
    ros-humble-cv-bridge \
    ros-humble-rosbridge-suite \
    ros-humble-vision-opencv \
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

WORKDIR /woeden_agent

RUN wget https://bootstrap.pypa.io/get-pip.py
RUN python3 get-pip.py
RUN python3 -m pip install stream-zip imageio pandas rosbags

COPY scripts /woeden_agent/scripts
COPY certs /woeden_agent/certs
COPY src /woeden_agent/src
COPY docker/entrypoint.sh /woeden_agent/

RUN mkdir -p /.ros/log
RUN chmod -R 777 /.ros
RUN . /opt/ros/humble/setup.bash && colcon build
RUN rm -rf /woeden_agent/src/

ENV HOME /
ENV MQTT_HOST ssl://mqtt.woeden.com
ENV FASTRTPS_DEFAULT_PROFILES_FILE /super_client_configuration_file.xml

CMD ["./entrypoint.sh"]
