# Quick Start — Control RH5DG2 Hand via ROS2

Minimal guide. Install → configure → run → send commands.

---

## 0. Set Workspace Path

After extracting the package, point `INSPIRE_WS` at the `inspire_ws` directory.
Every path below is relative to it, so this works regardless of where you extracted.

```bash
# Run from the directory where you extracted the zip:
export INSPIRE_WS="$(pwd)/inspire_ws"

# ...or set it explicitly:
export INSPIRE_WS=/path/to/inspire_ws
```

Add the `export` line to `~/.bashrc` so it persists in new shells.

## 1. Prerequisites

- Ubuntu 22.04+
- ROS2 Humble installed and sourced (`source /opt/ros/humble/setup.bash`)
- RH5DG2 hand connected via USB-to-RS485 adapter

## 2. Install Dependencies

```bash
cd "$INSPIRE_WS"
chmod +x install_dependencies.sh
./install_dependencies.sh
```

Script installs: build tools, Boost, yaml-cpp, spdlog, dialout group.

If dialout group was just added:

```bash
newgrp dialout    # or log out + back in
```

If ROS2 Humble not installed, script prints install instructions — follow them, then re-run.

## 3. Build ROS2 Workspace

```bash
cd "$INSPIRE_WS/src/ros2"
source /opt/ros/humble/setup.bash
colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2
source install/setup.bash
```

Tip: add `source "$INSPIRE_WS/src/ros2/install/setup.bash"` to `~/.bashrc` for new shells.

## 4. Identify Serial Device

```bash
ls -l /dev/ttyUSB*
```

Pick the port your RH5DG2 is on (typically `/dev/ttyUSB0`).

## 5. Configure Device

Edit `$INSPIRE_WS/src/ros2/src/driver/config/device_protocol_config.yaml`:

```yaml
protocol:
  type: RH5DG2_485        # RH5DG2 over RS485

devices:
  - name: hand_left
    port: /dev/ttyUSB0    # match step 4
    baudrate: 115200
    Hand_ID: 1            # device ID set on hand

logging:
  level: INFO
  file: logs/hand_control.log
  console: true
  file_enable: true
  max_file_size_mb: 10
  max_files: 5
```

`Hand_ID` must match the hand's actual ID. Default = 1. Mismatch = node rejects writes.

Re-run `colcon build` after editing config (or use launch arg to point at edited file).

## 6. Launch Node

Single device:

```bash
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py \
  device_name:=hand_left
```

Node spins at 50Hz, reads `angleAct`, publishes state.

## 7. Control Hand

RH5DG2 has **13 joints**. `joint_values` length must be 13.

### Send Angle Command (Topic)

> **Safety:** RH5DG2 angle convention (verified — see `src/examples/RH5DG2.cpp`):
> - Range much wider than 0–1000. Working example uses per-finger range **965 (more open) ↔ 1800 (more closed)**, with abduction joints (indices 3, 5) held at `0`.
> - Direction (verified): example decreases values from 1800 → 965, fingers open. So **higher = closed, lower = open** for the bend joints.
> - All-`500` blanket command is unsafe on 13-DOF dexterous hand — thumb/abduction collide. Always start from a known-safe pose.

Safe init pose (lifted verbatim from `src/examples/RH5DG2.cpp` — verified on hardware):

```bash
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [1800,1800,1800,0,1800,0,1900,1900,1900,1900,1750,1600,2080]}"
```

Slightly open from init (decrement bend joints toward 965):

```bash
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [1500,1500,1500,0,1500,0,1900,1900,1900,1900,1750,1600,2080]}"
```

Fully open bend joints (min from example, abduction still 0, thumb cluster unchanged):

```bash
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [965,965,965,0,965,0,1900,1900,1900,1900,1750,1600,2080]}"
```

Joint map (inferred from example): indices 0–7 = finger bend joints (with 3, 5 = abduction held at 0), 8–12 = thumb (bend/rotation/abduction cluster). Only sweep indices 0,1,2,4 within `[965, 1800]` until you've mapped your unit. Don't change indices 3, 5, 8–12 without checking the datasheet.

### Read Actual Angles (Topic)

```bash
ros2 topic echo /hand_left/angle_actual
```

### Service Calls

```bash
# Set angles via service — verified safe init pose
ros2 service call /hand_left/set_angle rh5dg2_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [1800,1800,1800,0,1800,0,1900,1900,1900,1900,1750,1600,2080]}"

# Read error code
ros2 service call /hand_left/get_errorCode rh5dg2_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"

# Change device ID
ros2 service call /hand_left/set_id rh5dg2_interfaces/srv/Setid \
  "{hand_id: 1, device_id: 2}"
```

List all services:

```bash
ros2 service list | grep hand_left
ros2 topic list | grep hand_left
```

## 8. Inspect Interface

Check msg/srv field layout:

```bash
ros2 interface show rh5dg2_interfaces/msg/SetAngle1
ros2 interface show rh5dg2_interfaces/srv/Setangle
ros2 interface list -p rh5dg2_interfaces
```

## 9. Troubleshooting

| Symptom | Check |
|---------|-------|
| Permission denied on `/dev/ttyUSB0` | `groups \| grep dialout`, then `newgrp dialout` |
| Node rejects commands (`accepted: false`) | `hand_id` matches `Hand_ID` in config |
| No response from hand | baudrate (115200), wiring, power, `lsof /dev/ttyUSB0` |
| `rh5dg2_interfaces` not found | rebuild with all 3 packages selected |
| Wrong joint_values length | RH5DG2 = 13 values; RH56F1 = 6 |
| `CMakeCache.txt directory ... is different` / `source directory ... does not exist` | Stale build artifacts from another machine. Wipe and rebuild: `rm -rf "$INSPIRE_WS"/src/ros2/{build,install,log}` then re-run `colcon build` |

Logs:

```bash
tail -f logs/hand_control.log
ros2 run inspire_control_ros2 inspire_control_node --ros-args --log-level debug
```

---

## Minimal Cheat Sheet

```bash
# One-time
./install_dependencies.sh
cd src/ros2 && colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2

# Every shell
source /opt/ros/humble/setup.bash
source "$INSPIRE_WS/src/ros2/install/setup.bash"

# Run
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py device_name:=hand_left

# Move — verified safe init pose (from src/examples/RH5DG2.cpp)
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [1800,1800,1800,0,1800,0,1900,1900,1900,1900,1750,1600,2080]}"
```
