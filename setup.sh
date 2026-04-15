#!/bin/bash
#若ros2 topic list 列不出话题时，可尝试重复执行ros2 daemon stop 和ros2 daemon start命令，或者重启终端 
echo "Setup unitree ros2 environment"
#source /opt/ros/jazzy/setup.bash #当source该文件出现不停输出的问题时，注释掉该行

# Optional: source local cyclonedds overlay only when it exists.
if [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth1" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
