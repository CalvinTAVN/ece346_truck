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

# --- Step 1: Initialize submodules ---
echo -e "${BLUE}[1/5] Initializing submodules...${NC}"
cd "$SCRIPT_DIR"
git submodule update --init --recursive
echo -e "${GREEN}Submodules initialized.${NC}"

# --- Step 2: Install udev rules ---
echo -e "${BLUE}[2/5] Installing udev rules (VESC, Pololu, SLabs)...${NC}"
if [ -d "$SCRIPT_DIR/udev" ]; then
    sudo cp $SCRIPT_DIR/udev/*.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo -e "${GREEN}udev rules installed. VESC will appear at /dev/sensors/vesc.${NC}"
else
    echo -e "${RED}udev directory not found.${NC}"
fi

# --- Step 3: Install Logitech F710 driver ---
echo -e "${BLUE}[3/5] Installing Logitech F710 driver...${NC}"
JETPACK_VERSION=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-core 2>/dev/null | cut -f1 -d'-' | cut -f1 -d'.')

if [ -d "$SCRIPT_DIR/drivers/logitech-f710-module" ]; then
    cd "$SCRIPT_DIR/drivers/logitech-f710-module"
    if [ "$JETPACK_VERSION" -ge 36 ] 2>/dev/null; then
        echo -e "${RED}JetPack 6 detected. Logitech F710 requires kernel replacement."
        echo -e "See drivers/logitech-f710-module/JetPack6/README.md for instructions.${NC}"
    else
        echo "Running install-module.sh..."
        sudo ./install-module.sh
        echo -e "${GREEN}Logitech F710 driver installed. Cold boot required for it to take effect.${NC}"
    fi
else
    echo -e "${RED}Logitech driver submodule not found. Run 'git submodule update --init --recursive' first.${NC}"
fi

# --- Step 4: Install ROS2 dependencies ---
echo -e "${BLUE}[4/5] Installing ROS2 dependencies...${NC}"
cd "$WS_DIR"
if command -v rosdep &> /dev/null; then
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
    echo -e "${GREEN}ROS2 dependencies installed.${NC}"
else
    echo -e "${RED}rosdep not found. Make sure ROS2 Foxy is sourced.${NC}"
fi

# --- Step 5: Build the workspace ---
echo -e "${BLUE}[5/5] Building ROS2 workspace...${NC}"
cd "$WS_DIR"
source /opt/ros/foxy/setup.bash 2>/dev/null || true
colcon build --symlink-install
echo -e "${GREEN}Build complete.${NC}"

echo ""
echo -e "${GREEN}=== Setup Complete ==="
echo -e "To use, run: source $WS_DIR/install/setup.bash${NC}"
