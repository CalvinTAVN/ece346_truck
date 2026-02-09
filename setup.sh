#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(dirname $(dirname "$SCRIPT_DIR"))

echo -e "${BLUE}=== ece346_truck Setup ==="
echo -e "Repo:      $SCRIPT_DIR"
echo -e "Workspace: $WS_DIR${NC}"
echo ""

# --- Step 1: Install ROS2 Foxy ---
echo -e "${BLUE}[1/7] Checking ROS2 Foxy installation...${NC}"

# Refresh ROS GPG key (fixes expired key errors)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

if [ -d "/opt/ros/foxy" ]; then
    echo -e "${GREEN}ROS2 Foxy already installed.${NC}"
else
    echo "Installing ROS2 Foxy..."

    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt install -y software-properties-common curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-foxy-desktop

    echo -e "${GREEN}ROS2 Foxy installed.${NC}"
fi

source /opt/ros/foxy/setup.bash

# --- Step 2: Install build tools ---
echo -e "${BLUE}[2/7] Installing build tools...${NC}"
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init || true
fi
rosdep update
echo -e "${GREEN}Build tools installed.${NC}"

# --- Step 3: Initialize submodules ---
echo -e "${BLUE}[3/7] Initializing submodules...${NC}"
cd "$SCRIPT_DIR"
git submodule update --init --recursive
echo -e "${GREEN}Submodules initialized.${NC}"

# --- Step 4: Install udev rules ---
echo -e "${BLUE}[4/7] Installing udev rules (VESC, Pololu, SLabs)...${NC}"
if [ -d "$SCRIPT_DIR/udev" ]; then
    sudo cp $SCRIPT_DIR/udev/*.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo -e "${GREEN}udev rules installed. VESC will appear at /dev/sensors/vesc.${NC}"
else
    echo -e "${RED}udev directory not found.${NC}"
fi

# --- Step 5: Install Logitech F710 driver ---
echo -e "${BLUE}[5/7] Installing Logitech F710 driver...${NC}"
JETPACK_VERSION=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-core 2>/dev/null | cut -f1 -d'-' | cut -f1 -d'.')

if [ -d "$SCRIPT_DIR/drivers/logitech-f710-module" ]; then
    cd "$SCRIPT_DIR/drivers/logitech-f710-module"
    if [ "$JETPACK_VERSION" -ge 36 ] 2>/dev/null; then
        echo -e "${RED}JetPack 6 detected. Logitech F710 requires kernel replacement."
        echo -e "See drivers/logitech-f710-module/JetPack6/README.md for instructions.${NC}"
    else
        echo "Running install-module.sh..."
        sudo ./install-module.sh
        # Make the module load automatically on boot
        if ! grep -q "hid-logitech" /etc/modules 2>/dev/null; then
            echo "hid-logitech" | sudo tee -a /etc/modules
        fi
        sudo depmod -a
        echo -e "${GREEN}Logitech F710 driver installed and set to load on boot. Cold boot required.${NC}"
    fi
else
    echo -e "${RED}Logitech driver submodule not found.${NC}"
fi

# --- Step 6: Install ROS2 dependencies ---
echo -e "${BLUE}[6/7] Installing ROS2 dependencies...${NC}"
cd "$WS_DIR"
sudo apt install -y \
    ros-foxy-joy-linux \
    ros-foxy-ackermann-msgs \
    ros-foxy-joy-teleop \
    ros-foxy-rosbridge-server \
    ros-foxy-control-msgs \
    ros-foxy-diagnostic-updater \
    ros-foxy-serial-driver \
    ros-foxy-urg-node \
    2>/dev/null || true
rosdep install --from-paths src -i -y || true
echo -e "${GREEN}ROS2 dependencies installed.${NC}"

# --- Step 7: Build the workspace ---
echo -e "${BLUE}[7/7] Building ROS2 workspace...${NC}"
cd "$WS_DIR"
colcon build --symlink-install
echo -e "${GREEN}Build complete.${NC}"

echo ""
echo -e "${GREEN}=== Setup Complete ==="
echo -e "To use, run: source $WS_DIR/install/setup.bash"
echo -e "NOTE: Cold boot required for Logitech F710 driver to take effect.${NC}"
