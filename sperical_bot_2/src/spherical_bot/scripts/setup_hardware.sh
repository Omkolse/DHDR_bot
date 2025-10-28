#!/bin/bash
#
# Spherical Bot - Hardware Setup Script
# Configures GPIO, I2C, and system settings for Raspberry Pi Zero 2W
#

set -e  # Exit on any error

echo "🔧 Spherical Bot Hardware Setup"
echo "================================"

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo "⚠️  WARNING: Not running on Raspberry Pi. Some setup steps will be skipped."
fi

# Function to print status
print_status() {
    echo "✅ $1"
}

# Function to print error
print_error() {
    echo "❌ $1"
}

# Enable I2C
echo "Enabling I2C interface..."
if ! grep -q "i2c-dev" /etc/modules; then
    echo "i2c-dev" | sudo tee -a /etc/modules
fi

sudo raspi-config nonint do_i2c 0
print_status "I2C enabled"

# Enable SPI (for potential future sensors)
echo "Enabling SPI interface..."
sudo raspi-config nonint do_spi 0
print_status "SPI enabled"

# Set CPU governor to performance for real-time control
echo "Setting CPU governor to performance..."
if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
    print_status "CPU governor set to performance"
else
    echo "⚠️  CPU governor control not available"
fi

# Increase USB read timeout for sensors
echo "Configuring USB settings..."
echo 1000 | sudo tee /sys/module/usbcore/parameters/autosuspend 2>/dev/null || true

# Configure real-time priorities
echo "Configuring real-time priorities..."
if ! grep -q "@sphericalbot" /etc/security/limits.conf; then
    echo "@sphericalbot - rtprio 99" | sudo tee -a /etc/security/limits.conf
    echo "@sphericalbot - memlock unlimited" | sudo tee -a /etc/security/limits.conf
    print_status "Real-time priorities configured"
fi

# Set up GPIO permissions
echo "Setting up GPIO permissions..."
sudo usermod -a -G gpio $USER 2>/dev/null || true

# Configure swap for memory management (important for Pi Zero 2W with 512MB RAM)
echo "Configuring swap file..."
if command -v dphys-swapfile >/dev/null 2>&1; then
    sudo dphys-swapfile swapoff
    sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=512/' /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    print_status "Swap file configured (512MB)"
else
    echo "⚠️  dphys-swapfile not available"
fi

# Reduce GPU memory (set to minimum for headless operation)
echo "Reducing GPU memory..."
if [ -f /boot/config.txt ]; then
    if ! grep -q "gpu_mem=16" /boot/config.txt; then
        echo "gpu_mem=16" | sudo tee -a /boot/config.txt
        print_status "GPU memory set to 16MB"
    fi
fi

# Disable unnecessary services to save resources
echo "Disabling unnecessary services..."
sudo systemctl disable bluetooth 2>/dev/null || true
sudo systemctl disable avahi-daemon 2>/dev/null || true
sudo systemctl disable triggerhappy 2>/dev/null || true
sudo systemctl disable cups 2>/dev/null || true
sudo systemctl disable cups-browsed 2>/dev/null || true

print_status "Unnecessary services disabled"

# Set up udev rules for consistent device naming (if needed)
echo "Setting up udev rules..."
if [ ! -f /etc/udev/rules.d/99-spherical-bot.rules ]; then
    sudo tee /etc/udev/rules.d/99-spherical-bot.rules > /dev/null <<EOF
# Spherical Bot UDEV rules
# IMU device
SUBSYSTEM=="i2c-dev", KERNEL=="i2c-1", GROUP="i2c", MODE="0660"

# GPIO access
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"
EOF
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    print_status "UDEV rules configured"
fi

# Create data directories
echo "Creating data directories..."
mkdir -p ~/spherical_bot_data/logs
mkdir -p ~/spherical_bot_data/calibration
mkdir -p ~/spherical_bot_data/maps

print_status "Data directories created"

# Install required system packages
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    python3-pip \
    i2c-tools \
    python3-smbus \
    python3-rpi.gpio \
    python3-psutil \
    fbi \
    stress \
    sysstat

print_status "System dependencies installed"

# Test I2C detection
echo "Testing I2C detection..."
if command -v i2cdetect >/dev/null 2>&1; then
    echo "I2C devices detected:"
    i2cdetect -y 1 | grep -v "UU" || true
else
    echo "⚠️  i2cdetect not available"
fi

# Set executable permissions for scripts
echo "Setting script permissions..."
chmod +x ~/spherical_bot_ws/src/spherical_bot/scripts/*.sh

print_status "Script permissions set"

echo ""
echo "🎉 Hardware setup completed successfully!"
echo ""
echo "Next steps:"
echo "1. Reboot the system: sudo reboot"
echo "2. Build the ROS package: colcon build --packages-select spherical_bot"
echo "3. Source the workspace: source install/setup.bash"
echo "4. Launch the bot: ros2 launch spherical_bot spherical_bot.launch.py"
echo ""
echo "For troubleshooting, see: docs/troubleshooting.md"