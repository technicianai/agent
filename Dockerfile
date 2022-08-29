FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install openssl

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

WORKDIR /woeden_monitor

COPY . /woeden_monitor/
RUN . /opt/ros/humble/setup.bash && colcon build

ENV HOME /
ENV HOST 3.140.168.45

RUN rm -rf /woeden_monitor/src/

CMD ["./ros-entrypoint.sh"]
