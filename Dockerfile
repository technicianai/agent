FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN mkdir downloads

# Install Paho MQTT C
RUN cd downloads && \
    git clone https://github.com/eclipse/paho.mqtt.c.git && \
    cd paho.mqtt.c && \
    git checkout v1.3.8 && \
    cmake -Bbuild -H. \
        -DPAHO_ENABLE_TESTING=OFF \
        -DPAHO_BUILD_STATIC=ON \
        -DPAHO_WITH_SSL=ON \
        -DPAHO_HIGH_PERFORMANCE=ON && \
    cmake --build build/ --target install && \
    ldconfig

# Install Paho MQTT Cpp
RUN cd downloads && \
    git clone https://github.com/eclipse/paho.mqtt.cpp && \
    cd paho.mqtt.cpp && \
    cmake -Bbuild -H. \
        -DPAHO_BUILD_STATIC=ON \
        -DPAHO_BUILD_SAMPLES=TRUE && \
    cmake --build build/ --target install && \
    ldconfig

WORKDIR /woden-monitor

COPY . /woden-monitor/ws/src/woden-monitor
COPY ros-entrypoint.sh /woden-monitor/
COPY certs /woden-monitor/

RUN . /opt/ros/humble/setup.bash \
    && cd ws \
    && colcon build

CMD ["./ros-entrypoint.sh"]
