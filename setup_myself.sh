#!/usr/bin/env zsh
#若ros2 topic list 列不出话题时，可尝试重复执行ros2 daemon stop 和ros2 daemon start命令，或者重启终端 
echo "Setup unitree ros2 environment"

# 加载 ROS2 基础环境。
source /opt/ros/jazzy/setup.zsh #当source该文件出现不停输出的问题时，注释掉该行

# 如存在当前工作区 overlay，则一并加载。
if [[ -f "$HOME/unitree_ros2/install/setup.zsh" ]]; then
        source "$HOME/unitree_ros2/install/setup.zsh"
fi

# 可选：若本地 CycloneDDS overlay 存在则加载。
if [[ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.zsh" ]]; then
        source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.zsh"
elif [[ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]]; then
        source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 允许手动覆盖：
#   export ROBOT_IP=192.168.123.164
#   export ROS2_NET_IFACE=eth2
ROBOT_IP=${ROBOT_IP:-192.168.123.164}
ROS2_NET_IFACE=${ROS2_NET_IFACE:-}

if [[ -z "$ROS2_NET_IFACE" ]]; then
        ROS2_NET_IFACE=$(ip route get "$ROBOT_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++){if($i=="dev"){print $(i+1); exit}}}')
fi

if [[ -z "$ROS2_NET_IFACE" ]]; then
        ROS2_NET_IFACE=$(ip -o -4 addr show up scope global | awk 'NR==1{print $2}')
fi

if [[ -z "$ROS2_NET_IFACE" ]]; then
        echo "[ERROR] No usable network interface found for CycloneDDS."
        return 1 2>/dev/null || exit 1
fi

# 直接使用内联 CycloneDDS 配置，避免生成运行时临时文件。
export CYCLONEDDS_URI="$(cat <<EOF
<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<CycloneDDS xmlns=\"https://cdds.io/config\">
    <Domain id=\"any\">
        <General>
            <Interfaces>
                <NetworkInterface name=\"$ROS2_NET_IFACE\" />
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer Address=\"$ROBOT_IP\" />
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF
)"

echo "ROBOT_IP=$ROBOT_IP"
echo "ROS2_NET_IFACE=$ROS2_NET_IFACE"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI=inline-xml"