FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    openssl \
    openssh-client \
    ros-humble-ackermann-msgs \
    ros-humble-rosbridge-suite \
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

# COPY src/topic_tools/ /woeden_agent/src/topic_tools/
# RUN . /opt/ros/humble/setup.bash && colcon build

# COPY src/woeden_agent/ /woeden_agent/src/woeden_agent/
# RUN . /opt/ros/humble/setup.bash && colcon build --packages-select woeden_agent

COPY src/ /woeden_agent/
RUN . /opt/ros/humble/setup.bash && colcon build

COPY bag_utils/ /woeden_agent/bag_utils/
RUN python3 bag_utils/get-pip.py && python3 -m pip install stream-zip

COPY certs/ /woeden_agent/certs/
COPY ros-entrypoint.sh /woeden_agent/

ENV HOME /
ENV HOST 3.140.168.45

RUN rm -rf /woeden_agent/src/

CMD ["./ros-entrypoint.sh"]
