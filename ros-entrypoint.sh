#!/bin/bash

. /opt/ros/humble/setup.bash
. install/setup.bash

if [[ ! -f "/woeden/id" ]]; then
    echo "We were unable to identify your robot. Did you run the setup script provided at https://woeden.com/?"
    exit 1
fi
mkdir -p /woeden/bags

ros2 run woeden_agent woeden_agent
