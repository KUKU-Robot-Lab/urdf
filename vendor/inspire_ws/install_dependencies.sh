#!/bin/bash
# Dexterous Hand Control System - Dependency Install Script
# For Ubuntu 22.04+

set -e  # exit on error

echo "=========================================="
echo "  Dexterous Hand Control System - Dependency Install Script"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
   echo "Error: do not run this script as root"
   exit 1
fi

# Detect OS
if [ ! -f /etc/os-release ]; then
    echo "Error: cannot detect operating system"
    exit 1
fi

. /etc/os-release

if [ "$ID" != "ubuntu" ] && [ "$ID" != "debian" ]; then
    echo "Warning: this script targets Ubuntu/Debian; other systems may need manual install"
    read -p "Continue? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "Detected OS: $PRETTY_NAME"
echo ""

# Update package list
echo "=== Step 1/6: Update package list ==="
sudo apt update
echo ""

# Install system dependencies
echo "=== Step 2/6: Install system dependencies ==="
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    gcc \
    g++ \
    make \
    libc6-dev

echo ""

# Check CMake version
CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d'.' -f1)
CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d'.' -f2)

if [ "$CMAKE_MAJOR" -lt 3 ] || ([ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -lt 10 ]); then
    echo "Warning: CMake version ($CMAKE_VERSION) is below required (3.10+)"
    echo "Recommend upgrading CMake or building from source"
else
    echo "✓ CMake version check passed: $CMAKE_VERSION"
fi
echo ""

# Install Boost
echo "=== Step 3/6: Install Boost ==="
sudo apt install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-dev

# Verify Boost install
if pkg-config --exists boost; then
    BOOST_VERSION=$(pkg-config --modversion boost)
    echo "✓ Boost installed: $BOOST_VERSION"
else
    echo "Warning: cannot verify Boost install, but it may have succeeded"
fi
echo ""

# Install yaml-cpp
echo "=== Step 4/6: Install yaml-cpp ==="
sudo apt install -y libyaml-cpp-dev

# Verify yaml-cpp install
if pkg-config --exists yaml-cpp; then
    YAML_VERSION=$(pkg-config --modversion yaml-cpp)
    echo "✓ yaml-cpp installed: $YAML_VERSION"
else
    echo "Warning: cannot verify yaml-cpp install, but it may have succeeded"
fi
echo ""

# Install spdlog
echo "=== Step 5/6: Install spdlog ==="
if sudo apt install -y libspdlog-dev; then
    echo "✓ spdlog installed"
else
    echo "Warning: apt install of spdlog failed, trying source build..."

    # Build spdlog from source
    cd /tmp
    if [ -d "spdlog" ]; then
        rm -rf spdlog
    fi

    git clone https://github.com/gabime/spdlog.git
    cd spdlog
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$(nproc)
    sudo make install
    cd /tmp
    rm -rf spdlog

    echo "✓ spdlog built and installed from source"
fi
echo ""

# Configure serial port permissions
echo "=== Step 6/6: Configure serial port permissions ==="
if groups | grep -q dialout; then
    echo "✓ User already in dialout group"
else
    sudo usermod -a -G dialout $USER
    echo "✓ User added to dialout group"
    echo "  Note: re-login or run 'newgrp dialout' for it to take effect"
fi
echo ""

# Check ROS2 install
echo "=== Check ROS2 install ==="
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "✓ ROS2 Humble detected"
    source /opt/ros/humble/setup.bash

    # Check ROS2 packages
    if ros2 pkg list | grep -q rclcpp; then
        echo "✓ ROS2 package check passed"
    else
        echo "Warning: ROS2 packages may be incomplete, recommend running:"
        echo "  sudo apt install -y ros-humble-desktop"
    fi
else
    echo "⚠ ROS2 Humble not installed"
    echo ""
    echo "Follow these steps to install ROS2 Humble:"
    echo ""
    echo "1. Set locale:"
    echo "   sudo apt install -y locales"
    echo "   sudo locale-gen en_US en_US.UTF-8"
    echo "   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8"
    echo ""
    echo "2. Add ROS2 source:"
    echo "   sudo apt install -y software-properties-common"
    echo "   sudo add-apt-repository universe"
    echo "   sudo apt update && sudo apt install -y curl gnupg lsb-release"
    echo "   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -"
    echo "   sudo sh -c 'echo \"deb [arch=\$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu \$(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2-latest.list'"
    echo ""
    echo "3. Install ROS2:"
    echo "   sudo apt update"
    echo "   sudo apt install -y ros-humble-desktop"
    echo "   sudo apt install -y ros-humble-rclcpp ros-humble-std-msgs ros-humble-std-srvs"
    echo "   sudo apt install -y ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime"
    echo "   sudo apt install -y python3-colcon-common-extensions python3-rosdep"
    echo ""
    echo "4. Initialize rosdep:"
    echo "   sudo rosdep init"
    echo "   rosdep update"
    echo ""
    echo "5. Set environment (add to ~/.bashrc):"
    echo "   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
    echo "   source ~/.bashrc"
fi
echo ""

# Summary
echo "=========================================="
echo "  Dependency install done!"
echo "=========================================="
echo ""
echo "Installed dependencies:"
echo "  ✓ System build tools (CMake, GCC, Make, etc.)"
echo "  ✓ Boost (system, thread)"
echo "  ✓ yaml-cpp"
echo "  ✓ spdlog"
echo "  ✓ Serial port permissions configured"
echo ""
echo "ROS2 side (packages built from source, not apt):"
echo "  • rh5dg2_interfaces / rh56f1_interfaces — model-specific msg/srv"
echo "  • inspire_control_ros2 — dexterous hand node (depends on the interface packages above)"
echo ""
echo "Next steps:"
echo "  1. If serial port permissions changed, re-login or run: newgrp dialout"
echo "  2. If ROS2 is not installed, follow the instructions above to install ROS2 Humble"
echo "  3. Build the ROS2 workspace (interface packages + node package, first build must include interfaces):"
echo "     cd src/ros2"
echo "     source /opt/ros/humble/setup.bash"
echo "     colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2"
echo "     source install/setup.bash"
echo "  When only changing node code, you can simplify to: colcon build --packages-select inspire_control_ros2"
echo ""
echo "Verify install:"
echo "  cmake --version"
echo "  g++ --version"
echo "  pkg-config --modversion boost"
echo "  pkg-config --modversion yaml-cpp"
echo ""
