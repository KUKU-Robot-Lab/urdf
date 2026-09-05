# Dexterous Hand Control System (Inspire / ROS2)

A multi-device dexterous hand control system based on C++ and ROS2. Communicates with multiple Inspire-series dexterous hands at the lower layer via RS485 / CANFD, etc. The node package name is **`inspire_control_ros2`**.

## Project Introduction

This project is a modular dexterous hand control system supporting:
- ✅ **Multi-device support**: Simultaneously controls multiple dexterous hand devices (e.g., left hand, right hand)
- ✅ **Multi-protocol support**: Supports multiple communication protocols (RH56F1_485, RH5DG2_485, etc.) via factory pattern
- ✅ **Dynamic configuration**: Device protocols and ROS2 topics/services configurable through YAML
- ✅ **Dual communication modes**: Topic (real-time control) and Service (on-demand call)
- ✅ **Asynchronous serial communication**: Based on Boost.Asio, with timeout and error handling
- ✅ **Unified logging system**: Global logger manager with file rotation and level control

## Project Structure

```
serial_control/
├── src/                          # Source code directory
│   ├── include/                  # Header files
│   │   ├── protocol.hpp          # Protocol abstract base class
│   │   ├── RH56F1_485_protocol.hpp
│   │   ├── RH5DG2_485_protocol.hpp
│   │   ├── serial_port.hpp       # Serial communication
│   │   ├── device_manager.hpp    # Device manager
│   │   ├── config_loader.hpp     # Config loader
│   │   ├── logger_manager.hpp    # Logger manager
│   │   ├── ring_buffer.hpp       # Ring buffer
│   │   └── protocol_factory.hpp  # Protocol factory
│   ├── src/                      # Source files
│   │   ├── protocol_factory.cpp
│   │   ├── RH56F1_485_protocol.cpp
│   │   ├── RH5DG2_485_protocol.cpp
│   │   ├── serial_port.cpp
│   │   ├── device_manager.cpp
│   │   ├── config_loader.cpp
│   │   ├── logger_manager.cpp
│   │   └── ring_buffer.cpp
│   ├── examples/                 # Example programs
│   │   └── main.cpp              # Multi-device parallel control example
│   ├── config/                   # Configuration files
│   │   ├── device_protocol_config.yaml    # Device protocol config
│   │   └── RH56F1.yaml, RH5DG2.yaml    # Device config examples
│   └── ros2/                          # ROS2 workspace (colcon top-level)
│       └── src/
│           ├── driver/                # Package inspire_control_ros2
│           │   ├── src/               # Node, RegisterController, model adapters
│           │   ├── include/
│           │   ├── config/            # device_protocol_config.yaml, ros2_controller_config*.yaml
│           │   └── launch/            # inspire_control_*.launch.py
│           └── interfaces/
│               ├── RH5DG2/            # Interface package rh5dg2_interfaces (13 DOF)
│               └── RH56F1/            # Interface package rh56f1_interfaces (6 DOF)
├── architecture.en.md             # Overall project architecture doc
├── module_usage.en.md             # Detailed module usage guide
├── dependencies.en.md             # Dependency list and install instructions
├── install_dependencies.sh        # Dependency install script (one-click)
└── README.en.md                   # This file
```

### ROS2 Interface Notes (After Refactor)

| Package | Purpose |
|---------|---------|
| **inspire_control_ros2** | Node and driver logic: `inspire_control_node`, `RegisterController`, `RH5DG2InterfaceAdapter` / `RH56F1InterfaceAdapter`. Config files installed in `share/inspire_control_ros2/config`. |
| **rh5dg2_interfaces** | RH5DG2 (13 DOF) dedicated `msg`/`srv`, e.g., `SetAngle1`, `GetAngleAct1`, `Setforce`, `Geterror`, etc. |
| **rh56f1_interfaces** | RH56 series (6 DOF) dedicated `msg`/`srv`. |

Set **`protocol.type`** in **`device_protocol_config.yaml`** (e.g., **`RH5DG2_485`**, **`RH56F1_485`**, **`RH5DG2_canfd`**). At startup the system automatically derives **`interfaces_profile`** (`RH5DG2` / `RH56F1`) and creates the corresponding adapter, binding to ROS types from **`rh5dg2_interfaces` or `rh56f1_interfaces`**.

Must be built together with the interface packages in the workspace (see [Build Project](#3-build-project)).

## Quick Start

> **💡 Quick install**: Recommended to use the automated install script for one-click dependency installation
> ```bash
> ./install_dependencies.sh
> ```
> Details in [Dependency Installation](#2-dependency-installation) section

### 1. Environment Requirements

- **OS**: Linux (Ubuntu 22.04+)
- **ROS2**: Humble or later
- **C++ Standard**: C++17
- **Compiler**: GCC 9+ or Clang 10+
- **Build tool**: CMake 3.10+

### 2. Dependency Installation

#### 2.1 System Dependencies

**Ubuntu/Debian**:

```bash
# Update package list
sudo apt update

# Install basic build tools
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl

# Install C++ compiler and toolchain
sudo apt install -y \
    gcc \
    g++ \
    make \
    libc6-dev
```

#### 2.2 ROS2 Dependencies

**Install ROS2 Humble (if not already installed)**:

```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 source
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Install ROS2 dev tools
sudo apt install -y \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Set up ROS2 environment (add to ~/.bashrc)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2.3 Third-party Library Dependencies

**Install Boost**:

```bash
# Install Boost dev libs (including Boost.Asio)
sudo apt install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-dev
```

**Install yaml-cpp**:

```bash
sudo apt install -y libyaml-cpp-dev
```

**Install spdlog**:

```bash
# Option 1: install via apt (recommended)
sudo apt install -y libspdlog-dev

# Option 2: build from source (if apt version not sufficient)
cd /tmp
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

#### 2.4 Serial Port Permissions

**Configure serial port access**:

```bash
# Option 1: add user to dialout group (recommended, permanent)
sudo usermod -a -G dialout $USER

# Option 2: temporary permission (reset after each reboot)
sudo chmod 666 /dev/ttyUSB0

# Note: Option 1 requires re-login to take effect
# Verify permission
groups | grep dialout
```

**Verify serial device**:

```bash
# List serial devices
ls -l /dev/ttyUSB*

# Check serial info
dmesg | grep ttyUSB
```

#### 2.5 Full Dependency List

**System-level deps**:
- `build-essential` - Basic build tools
- `cmake` (>= 3.10) - Build system
- `pkg-config` - Package config tool
- `gcc` / `g++` (>= 9) - C++ compiler
- `make` - Build tool

**ROS2 deps (apt)**:
- `ros-humble-desktop` - ROS2 desktop (or install `ros-humble-rclcpp` etc. as needed)
- `ros-humble-rclcpp` - ROS2 C++ client lib
- `ros-humble-std-msgs` - ROS2 standard messages
- `ros-humble-rosidl-default-generators` - ROS2 interface generator
- `ros-humble-rosidl-default-runtime` - ROS2 interface runtime
- `python3-colcon-common-extensions` - Colcon build tool extensions
- `python3-rosdep` - ROS dependency manager (optional)

**Repo's ROS2 workspace packages (built from source, not apt)**: `rh5dg2_interfaces`, `rh56f1_interfaces`, `inspire_control_ros2`. See "ROS2 Interface Notes" above and `dependencies.en.md`.

**Third-party libs**:
- `libboost-system-dev` - Boost system lib (includes Boost.Asio)
- `libboost-thread-dev` - Boost thread lib
- `libboost-dev` - Boost dev lib
- `libyaml-cpp-dev` - yaml-cpp dev lib
- `libspdlog-dev` - spdlog dev lib

**One-click install script**:

```bash
#!/bin/bash
# Full dependency install script

echo "=== Install system deps ==="
sudo apt update
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

echo "=== Install Boost ==="
sudo apt install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-dev

echo "=== Install yaml-cpp ==="
sudo apt install -y libyaml-cpp-dev

echo "=== Install spdlog ==="
sudo apt install -y libspdlog-dev

echo "=== Configure serial port permissions ==="
sudo usermod -a -G dialout $USER

echo "=== Dependency installation done ==="
echo "Note: serial port permission requires re-login to take effect"
echo "Run: newgrp dialout, or re-login"
```

#### 2.6 One-click Install Script (Recommended)

**Use the automated install script**:

```bash
cd /home/ubuntu/serial_control
chmod +x install_dependencies.sh
./install_dependencies.sh
```

The script will automatically:
- Detect the OS
- Install all system deps
- Install Boost, yaml-cpp, spdlog
- Configure serial port permissions
- Check ROS2 installation status
- Provide detailed install feedback

#### 2.7 Verify Installation

**Verify system deps**:

```bash
# Check CMake version
cmake --version  # should be >= 3.10

# Check GCC version
gcc --version    # should be >= 9

# Check G++ version
g++ --version    # should be >= 9
```

**Verify ROS2**:

```bash
# Check ROS2 environment
echo $ROS_DISTRO  # should show: humble

# Check ROS2 packages
ros2 pkg list | grep rclcpp

# Check colcon
colcon --version
```

**Verify third-party libs**:

```bash
# Check Boost
pkg-config --modversion boost

# Check yaml-cpp
pkg-config --modversion yaml-cpp

# Check spdlog (if installed via apt)
dpkg -l | grep spdlog
```

**Verify serial port permissions**:

```bash
# Check user group
groups | grep dialout

# Check serial devices
ls -l /dev/ttyUSB*  # user should have read/write permission
```

### 3. Build Project

#### Build core library (non-ROS2)

```bash
cd /home/ubuntu/serial_control/src
mkdir -p build && cd build
cmake ..
make
```

#### Build ROS2 workspace (interface packages + node package)

```bash
cd /home/ubuntu/serial_control/src/ros2
source /opt/ros/humble/setup.bash   # or your installed ROS2 distro
colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2
source install/setup.bash
```

When changing only node code, you can rebuild just `inspire_control_ros2`. But **on first clone or when interface packages change**, always include the two `*_interfaces` packages.

### 4. Configure Devices

Edit **`src/ros2/src/driver/config/device_protocol_config.yaml`** (or the `--device-config` path matching your launch):

```yaml
protocol:
  type: RH56F1_485

devices:
  - name: hand_left
    port: /dev/ttyUSB0
    baudrate: 115200
    Hand_ID: 1
```

### 5. Launch Node

#### Single-device mode

```bash
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py \
  device_name:=hand_left
```

#### Multi-device mode

```bash
ros2 launch inspire_control_ros2 inspire_control_multi_device.launch.py
```

### 6. Usage Examples

Examples below assume **`protocol.type`** is RH5DG2 series (**13** joints). For **RH56F1** series, change the package name to **`rh56f1_interfaces`** and **`joint_values` length to 6**. You can also use `ros2 interface show <pkg>/<type>` to inspect fields.

**`hand_id` binding with node**: The **`hand_id`** in inbound Topic/Service messages must match the device's **`Hand_ID`** in **`device_protocol_config.yaml`**, otherwise the node rejects register writes (`accepted: false`) or ignores subscription callbacks. **`hand_id: 0`** is treated as unspecified and still accepted (compat for callers that don't specify id).

#### Publish control command (topic mode)

```bash
# Angle command (adjust values per site calibration)
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"
```

#### Subscribe state data (topic mode)

```bash
ros2 topic echo /hand_left/angle_actual
```

#### Call service (service mode)

```bash
# Angle set service (maps to register angleSet)
ros2 service call /hand_left/set_angle rh5dg2_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

# Read error code (example)
ros2 service call /hand_left/get_errorCode rh5dg2_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"

# Set device comm ID
ros2 service call /hand_left/set_id rh5dg2_interfaces/srv/Setid \
  "{hand_id: 1, device_id: 1}"
```

## Documentation

### Project Architecture

📖 **[architecture.en.md](architecture.en.md)**

Includes:
- Overall system architecture diagram
- Module relationships and data flow
- Threading model
- Startup flow
- Extension points

### Module Usage

📖 **[module_usage.en.md](module_usage.en.md)**

Includes:
- Module function summaries
- Main classes and functions
- Configuration parameters
- Usage examples
- Data flow and communication modes

### Dependency List

📖 **[dependencies.en.md](dependencies.en.md)**

Includes:
- Complete dependency list
- Version requirements
- Install commands
- Verification methods
- FAQ

### Protocol Format

📖 **[src/document/RH56F1_485协议格式说明.md](src/document/RH56F1_485协议格式说明.md)**

Includes:
- Read/write request format
- Read/write reply format
- Meaning of each byte
- Checksum calculation
- Complete examples

## Core Modules

### 1. Serial Communication Module (SerialPortBase)

Asynchronous serial communication based on Boost.Asio, supporting blocking read/write and timeout.

**Main features**:
- Async receive
- Blocking send
- Timeout read
- Thread-safe

### 2. Protocol Abstraction Layer (Protocol)

Protocol abstract base class defining a unified protocol interface. Supports multiple protocol implementations (RH56F1_485, RH5DG2_485, etc.).

**Main features**:
- Command building
- Response parsing
- Checksum validation
- Register read/write

### 3. Device Manager (DeviceManager)

Manages multiple serial port devices, maintains port-to-device-object mapping.

**Main features**:
- Add/remove devices
- Query devices
- Multi-device management

### 4. ROS2 Controller (RegisterController)

ROS2 device control node, using **`rh5dg2_interfaces` / `rh56f1_interfaces`** message/service types via **`InterfaceAdapter`**.

**Main features**:
- Topic: subscribe commands, publish state (message types derived from `protocol.type` in **`device_protocol_config.yaml`**)
- Service: each function uses its own `.srv` — no more unified Register service
- Timer loop: 50Hz default (`update_rate` configurable)

### 5. Configuration System (ConfigLoader)

Loads config from YAML files, supports device config and logging config.

**Main features**:
- Device config loading
- Protocol object creation
- Logging system config

### 6. Logging System (LoggerManager)

Unified logging management based on spdlog.

**Main features**:
- Console and file output
- Log rotation
- Level control
- Thread-safe

## Communication Modes

### Topic Mode

**Characteristics**:
- High real-time
- Suitable for continuous control
- Timer loop reads and publishes

**Use cases**:
- Real-time angle control
- Real-time force control
- State monitoring

### Service Mode

**Characteristics**:
- On-demand call
- Not part of timer loop
- Suitable for single operations

**Use cases**:
- Device config (ID, baudrate, etc.)
- Error query
- State query

## Configuration Files

### Device Protocol Config (device_protocol_config.yaml)

```yaml
protocol:
  type: RH56F1_485

devices:
  - name: hand_left
    port: /dev/ttyUSB0
    baudrate: 115200
    Hand_ID: 1

logging:
  level: DEBUG
  file: logs/hand_control.log
  console: true
  file_enable: true
  max_file_size_mb: 10
  max_files: 5
```

### ROS2 Controller Config (ros2_controller_config.yaml)

```yaml
device_nodes:
  - device: hand_left
    update_rate: 50
    publish_header:
      frame_id: "hand_left"
    joint_names:
      - "hand_left/joint_0"
      # ... 13 items total (RH5DG2) or 6 items (RH56F1)

    topics:
      - name: angle_control
        registers:
          write: ["angleSet"]
          read: ["angleAct"]
        command_topic: "/hand_left/angle_set"
        state_topic: "/hand_left/angle_actual"

    services:
      - register_name: "angleSet"
        set_service_name: "/hand_left/set_angle"
        is_write_register: true
```

## FAQ

### 1. Dependency Install Issues

#### CMake version too low

```bash
cmake --version

# If < 3.10, upgrade CMake
# Ubuntu 22.04 default CMake usually meets the requirement
# If you need to upgrade, build from source or use snap
sudo snap install cmake --classic
```

#### Boost not found

```bash
pkg-config --modversion boost

# If not found, reinstall
sudo apt install --reinstall libboost-system-dev libboost-thread-dev libboost-dev

# Check library file locations
dpkg -L libboost-system-dev | grep .so
```

#### yaml-cpp not found

```bash
pkg-config --modversion yaml-cpp

# If not found, reinstall
sudo apt install --reinstall libyaml-cpp-dev

# Check library file locations
dpkg -L libyaml-cpp-dev | grep .so
```

#### spdlog not found

```bash
# Option 1: install via apt (recommended)
sudo apt install libspdlog-dev

# Option 2: build from source
cd /tmp
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

#### ROS2 not installed or wrong version

```bash
echo $ROS_DISTRO

# If unset, install ROS2 Humble
# See "Install ROS2 Dependencies" section above

# If wrong version, uninstall old version then reinstall
```

#### Headers not found at build time

```bash
# Check header file locations
dpkg -L libboost-dev | grep include
dpkg -L libyaml-cpp-dev | grep include

# If not found, reinstall dev packages
sudo apt install --reinstall libboost-dev libyaml-cpp-dev
```

### 2. Serial Port Permission Issues

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Re-login to take effect, or apply now
newgrp dialout

# Verify
groups | grep dialout

# Or set temporary permission
sudo chmod 666 /dev/ttyUSB0
```

### 3. Device Not Found

- Check serial devices: `ls -l /dev/ttyUSB*`
- Check port path in config file
- Check device is connected
- Check USB-serial driver: `lsmod | grep usbserial`

### 4. Communication Timeout

- Check baudrate config
- Check device ID (Hand_ID) config
- Check serial connection
- Check log file for errors
- Check serial port not held by another process: `lsof /dev/ttyUSB0`

### 5. ROS2 Node Won't Start

- Check config file path
- Check ROS2 env: `source install/setup.bash`
- Check ROS2 packages built: `colcon list`
- View logs: `ros2 run inspire_control_ros2 inspire_control_node --ros-args --log-level debug`
- Check node running: `ros2 node list`

### 6. Build Errors

#### ROS2 package not found

```bash
# Ensure ROS2 env is sourced
source /opt/ros/humble/setup.bash

# Check ROS2 packages
ros2 pkg list | grep rclcpp
```

#### Link errors

```bash
# Check library files exist
ldconfig -p | grep boost
ldconfig -p | grep yaml
ldconfig -p | grep spdlog

# Update dynamic linker cache
sudo ldconfig
```

#### CMake can't find packages

```bash
# Check pkg-config path
echo $PKG_CONFIG_PATH

# If empty, add default paths
export PKG_CONFIG_PATH=/usr/lib/pkgconfig:/usr/local/lib/pkgconfig
```

## Extension Development

### Add New Protocol

1. Create new protocol class inheriting `Protocol`
2. Implement all pure virtual functions
3. Register with `REGISTER_PROTOCOL` macro
4. Specify protocol type in config file

### Add New Register

1. Add register address (and read length, etc.) in the protocol's `REGISTER_MAP`
2. Add dedicated `srv`/`msg` in the model's interfaces package (if exposing externally)
3. Wire up the register in **`(device)_interface_adapter.cpp`**
4. Add `topics` or `services` entry in **`ros2_controller_config.yaml`**

### Add New Device

1. Add device config in `device_protocol_config.yaml`
2. Add device node config in `ros2_controller_config.yaml`
3. System will auto-recognize and start it

---

**Doc version**: v1.0
**Last updated**: 2026-05-12
