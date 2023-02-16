# woeden-agent

Built on ROS 2 and MQTT, this agent provides monitoring, remote control, and data collection capabilities out-of-the-box from the convenience of woeden.com.

## Getting started

To begin using this agent, begin by downloading and running our bash script on your personal computer or robot. This will register your robot within our system, providing it both an identifier and password that it can use to connect over MQTT.

```
$ wget https://woeden.s3.us-east-2.amazonaws.com/setup.bash
$ bash setup.bash
```

Next, simply clone this repository and begin running the agent.

```
$ git clone git@github.com:woedeninc/woeden-agent.git
$ cd woeden-agent
$ docker-compose up --build
```

To deploy:
`make deploy`

You can potentially get some errors. If they look docker related, try `make install` to set up the builder driver. It's also important to have an .env file so if you see complaints about unset envs need to get that.
