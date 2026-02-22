#!/bin/bash
# Setup CycloneDDS with unicast discovery for multi-machine ROS2
# Usage: source setup_cyclone.sh <peer_ip> <domain_id>
# Example (on Jetson):  source setup_cyclone.sh 192.168.1.100 5
# Example (on Host):    source setup_cyclone.sh 192.168.1.200 5

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: source setup_cyclone.sh <peer_ip> <domain_id>"
    echo "  peer_ip:   IP address of the other machine"
    echo "  domain_id: ROS_DOMAIN_ID (unique per group, 0-101)"
    return 1 2>/dev/null || exit 1
fi

PEER_IP="$1"
DOMAIN_ID="$2"
CONFIG_FILE="/tmp/cyclonedds.xml"

# Source ROS2 Foxy if available
if [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
fi

# Source workspace install if available (check relative to this script)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/install/setup.bash"
elif [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Get own IP for local discovery
OWN_IP=$(hostname -I | awk '{print $1}')

cat > "$CONFIG_FILE" <<EOF
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="${OWN_IP}"/>
        <Peer address="${PEER_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${CONFIG_FILE}
export ROS_DOMAIN_ID=${DOMAIN_ID}

echo "CycloneDDS configured:"
echo "  Peer:          ${PEER_IP}"
echo "  ROS_DOMAIN_ID: ${DOMAIN_ID}"
echo "  RMW:           ${RMW_IMPLEMENTATION}"
echo ""
echo "Make sure ros-foxy-rmw-cyclonedds-cpp is installed:"
echo "  sudo apt install ros-foxy-rmw-cyclonedds-cpp"
