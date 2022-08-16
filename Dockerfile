FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN mkdir downloads
RUN cd downloads && \
    git clone https://github.com/eclipse/paho.mqtt.c.git && \
    cd paho.mqtt.c && \
    make && \
    make install

WORKDIR /woeden_monitor

COPY . /woeden_monitor/
RUN . /opt/ros/humble/setup.bash && colcon build

ENV HOME /

CMD ["./ros-entrypoint.sh"]
