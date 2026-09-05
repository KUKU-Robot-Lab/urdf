# Dependency List

This document lists the dependencies needed by the project: **system / apt-installed libraries** and **ROS2 interface packages built alongside this repo**.

## System Requirements

- **OS**: Ubuntu 22.04+ (the `install_dependencies.sh` script targets Debian/Ubuntu)
- **Architecture**: x86_64 / amd64 (common)

## Build Tools

| Dependency | Version | Install Command |
|------------|---------|-----------------|
| CMake | >= 3.10 | `sudo apt install cmake` |
| GCC/G++ | >= 9 | `sudo apt install gcc g++` |
| Make | latest | `sudo apt install make` |
| pkg-config | latest | `sudo apt install pkg-config` |

## ROS2 Dependencies (apt / official source)

Workspace build and runtime requires a base ROS2 environment (using **Humble** as example; replace path/package prefix per your distribution).

| Dependency | Description | Typical Install Command |
|------------|-------------|-------------------------|
| ROS2 Humble (or Jazzy, etc.) | Desktop or minimal both work | `sudo apt install ros-humble-desktop` |
| rclcpp | C++ client library | `sudo apt install ros-humble-rclcpp` |
| std_msgs | Standard messages | `sudo apt install ros-humble-std-msgs` |
| rosidl_default_generators | Interface code generation | `sudo apt install ros-humble-rosidl-default-generators` |
| rosidl_default_runtime | Interface runtime | `sudo apt install ros-humble-rosidl-default-runtime` |
| colcon | Workspace build | `sudo apt install python3-colcon-common-extensions` |
| rosdep | Dependency resolver (optional) | `sudo apt install python3-rosdep` |

Note: **ament_index_cpp**, **builtin_interfaces** etc. usually come with `rclcpp` / desktop meta-package, no need to list separately.

## In-workspace ROS2 Packages (built from source, not apt)

The repo contains the following packages under `src/ros2/src/` that must be built by **colcon** in the same workspace as **`inspire_control_ros2`**:

| Package | Path (in repo) | Description |
|---------|----------------|-------------|
| **rh5dg2_interfaces** | `interfaces/RH5DG2` | RH5DG2 (13 DOF) dedicated `.msg` / `.srv` |
| **rh56f1_interfaces** | `interfaces/RH56F1` | RH56 series (6 DOF) dedicated `.msg` / `.srv` |
| **inspire_control_ros2** | `driver` | Node executable `inspire_control_node`, depends on the two interface packages above |

**Declaration** (see `src/ros2/src/driver/package.xml`): `inspire_control_ros2` **depend** on `rh5dg2_interfaces`, `rh56f1_interfaces`, `rclcpp`, `std_msgs`.

Recommended first-build command:

```bash
cd /path/to/serial_control/src/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2
source install/setup.bash
```

## Third-party Library Dependencies (apt)

| Dependency | Version | Install Command | Purpose |
|------------|---------|-----------------|---------|
| Boost (system) | >= 1.65 | `sudo apt install libboost-system-dev` | Async serial (Asio) |
| Boost (thread) | >= 1.65 | `sudo apt install libboost-thread-dev` | Threads |
| Boost (dev meta) | >= 1.65 | `sudo apt install libboost-dev` | Headers and CMake |
| yaml-cpp | >= 0.6 | `sudo apt install libyaml-cpp-dev` | YAML config parsing |
| spdlog | >= 1.5 | `sudo apt install libspdlog-dev` | Logging |

## System Permissions

| Permission | Description | Config Command |
|------------|-------------|----------------|
| dialout group | Access `/dev/ttyUSB*` etc. serial ports | `sudo usermod -a -G dialout $USER` |

## Quick Install

### Option 1: Install Script (Recommended)

```bash
./install_dependencies.sh
```

Script installs **CMake/GCC/Boost/yaml-cpp/spdlog** and configures serial port group. **If ROS2 is not installed**, it prints the official installation guide. Interface packages and node package must be built by **colcon** in the workspace after ROS2 is installed.

### Option 2: Manual Install

See "Dependency Installation" section in [README.en.md](README.en.md); third-party library commands match the table above.

## Verify Installation

```bash
cmake --version
gcc --version
pkg-config --modversion boost
pkg-config --modversion yaml-cpp
dpkg -l | grep spdlog

echo $ROS_DISTRO
ros2 pkg prefix rclcpp

groups | grep dialout
```

Build verification:

```bash
cd src/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2
```

## Dependency Graph (Conceptual)

```
serial_control
├── Build system: CMake / GCC / Make / pkg-config
├── ROS2 (system): rclcpp, std_msgs, rosidl toolchain, colcon
├── Workspace ROS packages (source)
│   ├── rh5dg2_interfaces ──┐
│   ├── rh56f1_interfaces ──┼──► inspire_control_ros2 (node)
│   └── (device protocol stack .cpp links Boost/yaml-cpp/spdlog)
├── Third-party libs: Boost, yaml-cpp, spdlog
└── Permissions: dialout (serial port)
```

## FAQ

### Q: Building only `inspire_control_ros2` errors with `rh5dg2_interfaces` not found?

Build the two interface packages in the same workspace first, or use:

`colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2`

### Q: Topic type is not `RegisterData`?

After refactor, message definitions live in **`rh5dg2_interfaces` / `rh56f1_interfaces`** and are selected by **`interfaces_profile`**. Use `ros2 topic info`, `ros2 interface list -p` to verify.

---

**Doc version**: v1.0  
**Last updated**: 2026-05-12
