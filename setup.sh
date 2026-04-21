#!/bin/bash
#若ros2 topic list 列不出话题时，可尝试重复执行ros2 daemon stop 和ros2 daemon start命令，或者重启终端 
echo "Setup unitree ros2 environment"
#source /opt/ros/jazzy/setup.bash #当source该文件出现不停输出的问题时，注释掉该行

# Optional: source local cyclonedds overlay only when it exists.
if [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 自动选择 CycloneDDS 网卡：优先匹配目标 IP，其次按到目标 IP 的路由推断，最后兜底 eth1
TARGET_IP="192.168.123.164"
NET_IFACE=$(ip -o -4 addr show | awk -v target="$TARGET_IP" '$4 ~ ("^" target "/") {print $2; exit}')

if [ -z "$NET_IFACE" ]; then
    NET_IFACE=$(ip route get "$TARGET_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}')
fi

if [ -z "$NET_IFACE" ]; then
    NET_IFACE="eth1"
fi

echo "CycloneDDS network interface: $NET_IFACE (target IP: $TARGET_IP)"

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name=\"$NET_IFACE\" priority=\"default\" multicast=\"default\" />
                        </Interfaces></General></Domain></CycloneDDS>"
