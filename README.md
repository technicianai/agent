[![SensorSurf](assets/logo.png)](https://www.sensorsurf.com)

<br/>

<div align="center">
    <h1>SensorSurf</h1>
    <a href="https://github.com/sensorsurf/agent/blob/humble/LICENSE"><img src="https://img.shields.io/badge/License-AGPL_v3-blue.svg" /></a>
    <br />
    <br />
    <a href="https://gallery.ecr.aws/woeden/agent">Docker</a>
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
$ wget https://github.com/SensorSurf/woeden-scripts/blob/master/setup.bash
$ bash setup.bash
```

Once these steps are completed, you may proceed with either running our pre-built Docker container image or building from source.

## Running the Docker image

This one is easy. Run the command below to get started. If you do not wish for the container to always restart, then please be sure to remove the `--restart always` option.

```bash
$ docker run -d \
    --net=host \
    --ipc=host \
    --restart always \
    -v ~/woeden:/woeden \
    public.ecr.aws/woeden/agent:latest
```

### Docker Compose files

We have a few Docker Compose files which can be used to test various system configurations.
* `root.docker-compose.yml` allows you to test the most common configuration, where Docker can access all ROS topics locally as root.
* `nonroot.docker-compose.yml` allows you to test a configuration where the local ROS setup only permits access to the current, non-root user.
* `discovery.docker-compose.yml` allows you to test a ROS system running a discovery server.

## Building from source

Follow the steps below to build from source, starting with dependencies:

1. Install the [Eclipse Paho MQTT C](https://github.com/eclipse/paho.mqtt.c) library with the following set of commands.
```
$ git clone git@github.com:eclipse/paho.mqtt.c.git
$ cd paho.mqtt.c
$ mkdir build
$ cmake . -DPAHO_WITH_SSL=TRUE -Bbuild
$ make
$ sudo make install
```

2. Install a few dependencies with pip.
```
$ python3 -m pip install \
    stream-zip \
    imageio \
    pandas \
    rosbags
```

3. Source your ROS environment, if you haven't already done so.
```
$ source /opt/ros/humble/setup.bash
```

4. Clone this repository into your ROS workspace (or wherever you want it) and compile with the following commands.
```
$ git clone git@github.com:SensorSurf/agent.git
$ cd agent
$ git checkout humble
$ colcon build
```

5. Source your workspace install, and run the agent.
```
$ source install/setup.bash
$ ros2 launch launch.py
```
