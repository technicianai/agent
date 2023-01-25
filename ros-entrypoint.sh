#!/bin/bash

. /opt/ros/humble/setup.bash
. install/setup.bash

if [[ ! -f "/woeden/config" ]]; then
    echo "We were unable to identify your robot. Did you run the setup script provided at https://woeden.com/?"
    exit 1
fi
mkdir -p /woeden/bags

if [[ ! -z "${ROS_DISCOVERY_SERVER}" ]]; then
    HOST=$(echo "${ROS_DISCOVERY_SERVER}" | cut -d ":" -f 1)
    PORT=$(echo "${ROS_DISCOVERY_SERVER}" | cut -d ":" -f 2)
    echo """<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<profiles xmlns=\"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles\">
    <participant profile_name=\"super_client_profile\" is_default_profile=\"true\">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                    <discoveryServersList>
                        <RemoteServer prefix=\"44.53.00.5f.45.50.52.4f.53.49.4d.41\">
                            <metatrafficUnicastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>${HOST}</address>
                                        <port>${PORT}</port>
                                    </udpv4>
                                </locator>
                            </metatrafficUnicastLocatorList>
                        </RemoteServer>
                    </discoveryServersList>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>""" > /super_client_configuration_file.xml
else
    unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

ros2 run woeden_agent woeden_agent &
ros2 run woeden_agent trigger_worker.py &
ros2 run woeden_agent upload_worker.py
