#!/bin/bash
# =======================================================================
# GoPiGo3 ROS2 Dependencies Installation Script
# =======================================================================
# This script installs all required dependencies for running GoPiGo3 with
# ROS2 Humble on Ubuntu 22.04
# =======================================================================

set -e

echo "============================================"
echo "GoPiGo3 ROS2 Dependencies Installation"
echo "============================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}Note: Some installations require sudo privileges${NC}"
fi

# ========================
# System Dependencies
# ========================
echo -e "\n${GREEN}[1/6] Installing system dependencies...${NC}"
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-rpi.gpio \
    python3-smbus \
    i2c-tools \
    libgpiod-dev

# ========================
# ROS2 Packages
# ========================
echo -e "\n${GREEN}[2/6] Installing ROS2 packages...${NC}"
sudo apt install -y \
    ros-humble-rplidar-ros \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher

# ========================
# Python Dependencies
# ========================
echo -e "\n${GREEN}[3/6] Installing Python dependencies...${NC}"
pip3 install \
    spidev \
    smbus2

# ========================
# GoPiGo3 Library
# ========================
echo -e "\n${GREEN}[4/6] Setting up GoPiGo3 library...${NC}"
if [ ! -d "$HOME/GoPiGo3" ]; then
    echo "Cloning GoPiGo3 repository..."
    cd ~
    git clone https://github.com/DexterInd/GoPiGo3.git
    cd GoPiGo3/Software/Python
    sudo python3 setup.py install
else
    echo "GoPiGo3 already exists at ~/GoPiGo3"
fi

# ========================
# DI_Sensors Library
# ========================
echo -e "\n${GREEN}[5/6] Setting up DI_Sensors library...${NC}"
if [ ! -d "$HOME/DI_Sensors" ]; then
    echo "Cloning DI_Sensors repository..."
    cd ~
    git clone https://github.com/DexterInd/DI_Sensors.git
    cd DI_Sensors/Python
    sudo python3 setup.py install
else
    echo "DI_Sensors already exists at ~/DI_Sensors"
fi

# ========================
# Enable SPI and I2C
# ========================
echo -e "\n${GREEN}[6/6] Enabling SPI and I2C...${NC}"
if ! grep -q "^dtparam=spi=on" /boot/firmware/config.txt 2>/dev/null; then
    echo "Enabling SPI..."
    echo "dtparam=spi=on" | sudo tee -a /boot/firmware/config.txt
fi

if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt 2>/dev/null; then
    echo "Enabling I2C..."
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
fi

# Add user to required groups
echo "Adding user to gpio, i2c, and spi groups..."
sudo usermod -aG gpio,i2c,spi $USER

# ========================
# GPG3 Power Service
# ========================
echo -e "\n${GREEN}Installing gpg3_power service...${NC}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo cp "$SCRIPT_DIR/gpg3_power.service" /etc/systemd/system/
sudo chmod +x "$SCRIPT_DIR/gpg3_power.py"
sudo systemctl daemon-reload
sudo systemctl enable gpg3_power
sudo systemctl start gpg3_power

# ========================
# Create Dexter config directory
# ========================
echo -e "\n${GREEN}Creating Dexter config directory...${NC}"
mkdir -p ~/Dexter
if [ ! -f ~/Dexter/gpg3_config.json ]; then
    cat > ~/Dexter/gpg3_config.json << 'EOF'
{
    "wheel-diameter": 66.5,
    "wheel-base-width": 117,
    "ticks": 6,
    "motor_gear_ratio": 120
}
EOF
    echo "Created ~/Dexter/gpg3_config.json"
fi

# ========================
# Create maps directory
# ========================
mkdir -p ~/maps

# ========================
# Done
# ========================
echo -e "\n${GREEN}============================================${NC}"
echo -e "${GREEN}Installation complete!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "Please reboot for all changes to take effect:"
echo "  sudo reboot"
echo ""
echo "After reboot, build the ROS2 workspace:"
echo "  cd ~/nav2_ws"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo ""
echo "To test the GoPiGo3:"
echo "  ros2 launch gopigo3_driver gopigo3_bringup.launch.py"
echo ""
echo "To start SLAM mapping:"
echo "  ros2 launch gopigo3_driver slam.launch.py"
echo ""
