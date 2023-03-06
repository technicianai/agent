[![SensorSurf](assets/logo.png)](https://www.sensorsurf.com)

<br/>

<div align="center">
    <h1>SensorSurf</h1>
    <a href="https://github.com/sensorsurf/agent/blob/noetic/LICENSE"><img src="https://img.shields.io/badge/License-AGPL_v3-blue.svg" /></a>
    <br />
    <br />
    <a href="https://gallery.ecr.aws/woeden/ros1-agent">Docker</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://docs.woeden.com">Docs</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://sensorsurf.com/blogs">Blog</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://app.woeden.com/auth/register">Try it out</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://sensorsurf.com/contact">Contact us</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://linkedin.com/company/sensorsurf">LinkedIn</a>
  <br />
  <br />
</div>

<hr />

This repository contains the source code for the [SensorSurf](sensorsurf.com) agent which runs directly on a robot. The major capabilities this agent enables are:

* Recording data on events.
* Offloading on a poor connection.
* Maintaining a rolling buffer.
* Exposing a live view.

Using a local configuration, your robot is able to connect via MQTT to our backend using [Eclipse Paho](http://eclipse.org/paho) library.

## Getting started

Before running the agent, make sure you have completed both of the following prequisities:

1. Create an account on [SensorSurf.com](sensorsurf.com).
2. Run our setup script to configure your machine:

```bash
$ wget https://raw.githubusercontent.com/SensorSurf/woeden-scripts/master/setup.bash
$ bash setup.bash
```

Once these steps are completed, you may proceed with either running our pre-built Docker container image or building from source.

## Running the Docker image

This one is easy. Run the command below to get started. If you do not wish for the container to always restart, then please be sure to remove the `--restart always` option. We compile for both x86 and ARM processors, so please set the `--platform` option accordingly.

```bash
$ docker run -d \
    --net host \
    --platform amd64 \
    --restart always \
    -v ~/woeden:/woeden \
    public.ecr.aws/woeden/ros1-agent:latest
```

### Docker Compose file

You can run the following command to build the container locally and run alongside roscore and a demo talker node, if you wish to test any changes.

```bash
$ docker-compose -f docker/docker-compose.yml up --build
```

## Building from source

Follow the steps below to build from source, starting with dependencies:

1. Install ROS Noetic following the instructions [here](http://wiki.ros.org/noetic/Installation).

2. Install ROS 2 Foxy following the instructions [here](https://docs.ros.org/en/foxy/Installation.html).

3. Install non-ROS dependencies using the commands below.
```bash
$ sudo apt-get update && sudo apt-get install -y \
    nlohmann-json3-dev \
    openssl \
    openssh-client \
    uuid-dev \
    wget
```

4. Install the [Eclipse Paho MQTT C](https://github.com/eclipse/paho.mqtt.c) library with the following set of commands.
```
$ git clone git@github.com:eclipse/paho.mqtt.c.git
$ cd paho.mqtt.c
$ mkdir build
$ cmake . -DPAHO_WITH_SSL=TRUE -Bbuild
$ make
$ sudo make install
```

5. Install the [Eclipse Paho MQTT C++](https://github.com/eclipse/paho.mqtt.cpp) library with the following set of commands.
```
$ git clone git@github.com:eclipse/paho.mqtt.cpp.git
$ cd paho.mqtt.cpp
$ cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON
$ sudo cmake --build build/ --target install
```

6. Install the following ROS dependencies.
```
$ sudo apt-get update && sudo apt-get install -y \
    ros-foxy-cv-bridge \
    ros-foxy-vision-opencv \
    ros-foxy-rosbag2* \
    ros-foxy-can-msgs \
    ros-foxy-nmea-msgs \
    ros-noetic-rosbag \
    ros-noetic-rosbridge-server \
    ros-noetic-topic-tools \
    ros-noetic-rosbag-snapshot
```

7. Install some dependencies with pip. One of these requires that ROS is already sourced.
```
$ . /opt/ros/noetic/setup.bash
$ pip install stream-zip rosbags matplotlib rosbag-merge
```

8. Clone the repository and build it.
```
$ git clone git@github.com:SensorSurf/agent.git
$ cd agent
$ git checkout noetic
$ . /opt/ros/foxy/setup.bash
$ colcon build --packages-skip ros1_bridge
$ . /opt/ros/noetic/setup.bash
$ . install/setup.bash
$ MAKEFLAGS="-j10 -l1" colcon build \
    --symlink-install \
    --packages-select ros1_bridge \
    --cmake-force-configure \
    --executor sequential
```

9. Run our stack.
```
$ ros2 run woeden_agent woeden_agent & \
    ros2 run woeden_agent trigger_worker.py & \
    ros2 run woeden_agent upload_worker.py & \
    ros2 run ros1_bridge dynamic_bridge \
        --bridge-all-1to2-topics \
        --unidirectional-1to2
```
