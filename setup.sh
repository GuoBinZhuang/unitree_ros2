#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash

# Optional: source local cyclonedds overlay only when it exists.
if [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth2" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
