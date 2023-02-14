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

## Deployment

```
aws ecr-public get-login-password --region us-east-1 | docker login --username AWS --password-stdin public.ecr.aws/woeden
docker buildx build --platform=linux/amd64,linux/arm64 -t public.ecr.aws/woeden/ros1-agent:latest .
```
