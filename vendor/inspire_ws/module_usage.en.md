# Dexterous Hand Control Project — Module Usage Guide

This document details each module's function, usage, config parameters, and examples.

---

## Table of Contents

1. [Serial Communication Module (SerialPortBase)](#1-serial-communication-module-serialportbase)
2. [Protocol Abstraction Layer (Protocol)](#2-protocol-abstraction-layer-protocol)
3. [Device Manager (DeviceManager)](#3-device-manager-devicemanager)
4. [Config Loader (ConfigLoader)](#4-config-loader-configloader)
5. [Logger Manager (LoggerManager)](#5-logger-manager-loggermanager)
6. [Ring Buffer (RingBuffer)](#6-ring-buffer-ringbuffer)
7. [Protocol Factory (ProtocolFactory)](#7-protocol-factory-protocolfactory)
8. [ROS2 Controller (RegisterController)](#8-ros2-controller-registercontroller)
9. [ROS2 Main Node (inspire_control_node)](#9-ros2-main-node-inspire_control_node)

---

## 1. Serial Communication Module (SerialPortBase)

### 1.1 Function

`SerialPortBase` is the serial communication abstract base class, implementing async serial communication via Boost.Asio. Provides blocking read/write interface with timeout support. Internally uses an async receive thread to continuously receive data.

### 1.2 Main Classes and Functions

#### Class definition

```cpp
class SerialPortBase {
public:
    SerialPortBase(const std::string& port, unsigned int baudrate);
    ~SerialPortBase();
    
    // Blocking write
    size_t write(const std::vector<uint8_t>& data);
    
    // Blocking read (with timeout)
    std::vector<uint8_t> read(std::chrono::milliseconds timeout = std::chrono::milliseconds(25));
    
    // Non-blocking read (returns current buffer data)
    std::vector<uint8_t> readAvailable();
    
    // Clear receive buffer
    void clearBuffer();
    
    // Check serial port status
    bool isOpen() const;
};
```

### 1.3 Config Parameters

**Constructor parameters**:
- `port`: Serial device path (e.g., `/dev/ttyUSB0`)
- `baudrate`: Baud rate (e.g., 115200)

**Internal parameters**:
- `BUFFER_SIZE = 4096`: Receive buffer size

### 1.4 Usage Example

```cpp
#include "serial_port.hpp"

// Create serial object
auto serial = std::make_shared<SerialPortBase>("/dev/ttyUSB0", 115200);

// Check if serial is open
if (!serial->isOpen()) {
    std::cerr << "Serial open failed" << std::endl;
    return;
}

// Write data
std::vector<uint8_t> cmd = {0xEB, 0x90, 0x01, 0x04, 0x11, 0x28, 0x04, 0x0C, 0xXX};
size_t written = serial->write(cmd);

// Read data (blocking, 25ms timeout)
auto data = serial->read(std::chrono::milliseconds(25));

// Non-blocking read (returns current buffer data immediately)
auto available_data = serial->readAvailable();

// Clear receive buffer
serial->clearBuffer();
```

### 1.5 Data Flow and Communication

**Write flow**:
```
Call write() → Boost.Asio sync write → serial hardware
```

**Read flow**:
```
Async receive thread → continuously receives data → stores in receive_buffer_
Call read() → reads from receive_buffer_ → returns data
```

**Threading**:
- Main thread: calls `write()` and `read()`
- Async thread: continuously receives serial data, stores in buffer

---

## 2. Protocol Abstraction Layer (Protocol)

### 2.1 Function

`Protocol` is the protocol abstract base class defining the interface spec all protocol implementations must follow. Concrete protocols (e.g., `RH56F1_485_Protocol`) inherit this class and implement protocol-specific command building and response parsing.

### 2.2 Main Classes and Functions

#### Base class interface

```cpp
class Protocol {
public:
    // Set/get device ID
    void setDeviceId(uint8_t id);
    uint8_t getDeviceId() const;
    
    // Get register address
    virtual int getRegisterAddress(const std::string& register_name) const = 0;
    
    // Build commands
    virtual std::vector<uint8_t> buildReadCommand(int address, size_t length) = 0;
    virtual std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) = 0;
    
    // Parse response
    virtual std::pair<bool, std::vector<int>> parseResponse(RingBuffer& ringBuffer) = 0;
    virtual bool validateChecksum(const std::vector<uint8_t>& response) const = 0;
    
    // Touch data
    virtual std::pair<bool, TouchDataResult> parseTouchData(RingBuffer& ringBuffer, int version) = 0;
    
    // High-level interface
    virtual bool writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) = 0;
    virtual std::pair<bool, std::vector<int>> readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name, size_t length) = 0;
    virtual std::pair<bool, TouchDataResult> readTouchData(Device device, RingBuffer& ringBuffer, int version) = 0;
};
```

#### Concrete protocol implementations

**RH56F1_485_Protocol**:
- Implements RH56F1 series 485 communication protocol
- Supports dynamic read length
- Supports touch data parsing (versions 1 and 2)

**RH5DG2_485_Protocol**:
- Implements RH5DG2 series 485 communication protocol
- Similar to RH56F1 but different register addresses
- Supports `actionLibraryIndex` special handling

### 2.3 Config Parameters

**Device ID (Hand_ID)**:
- Range: 1–254
- Default: 1
- Purpose: distinguish multiple devices on the same serial bus

**Register map**:
- Each protocol class maintains a `REGISTER_MAP` mapping register names to addresses
- E.g., `{"angleSet": 1040, "angleAct": 1064}`

**Read length map**:
- `REGISTER_READ_LENGTH_MAP`: defines default read length per register
- E.g., `{"angleAct": 12, "errorCode": 2}`

### 2.4 Usage Example

```cpp
#include "RH56F1_485_protocol.hpp"
#include "serial_port.hpp"
#include "ring_buffer.hpp"

// Create protocol object
auto protocol = std::make_shared<RH56F1_485_Protocol>();

// Set device ID
protocol->setDeviceId(1);

// Create serial port and buffer
auto device = std::make_shared<SerialPortBase>("/dev/ttyUSB0", 115200);
RingBuffer ringBuffer(1024);

// Write register
std::vector<int> values = {100, 200, 300, 400, 500, 600};
bool success = protocol->writeRegister(device, "angleSet", values);

// Read register (length=0 means auto-determine length)
auto [success, values] = protocol->readRegister(device, ringBuffer, "angleAct", 0);

// Read touch data
auto [success, touchData] = protocol->readTouchData(device, ringBuffer, 1);
```

### 2.5 Data Flow and Communication

**Write flow**:
```
writeRegister() 
  → buildWriteCommand() (build command frame)
  → device->write() (send to serial)
  → readResponseWithLoop() (read reply)
  → parseResponse() (parse reply)
  → return success/failure
```

**Read flow**:
```
readRegister() 
  → buildReadCommand() (build command frame)
  → device->write() (send to serial)
  → readResponseWithLoop() (read reply)
  → parseResponse() (parse reply)
  → return values and success flag
```

**Frame format**:
- Request: `0xEB 0x90` + device ID + data length + command + address + data + checksum
- Reply: `0x90 0xEB` + device ID + data length + command + address + data + checksum

---

## 3. Device Manager (DeviceManager)

### 3.1 Function

`DeviceManager` manages multiple serial port devices, maintaining a port-to-device-object map. Each device corresponds to one `SerialPortBase` object.

### 3.2 Main Classes and Functions

```cpp
class DeviceManager {
public:
    // Add device
    void addDevice(const std::string& port, std::shared_ptr<Protocol> protocol, int baudRate);
    
    // Get device
    std::shared_ptr<SerialPortBase> getDevice(const std::string& port);
    
    // Remove device
    void removeDevice(const std::string& port);
};
```

### 3.3 Config Parameters

**addDevice parameters**:
- `port`: Serial device path (e.g., `/dev/ttyUSB0`)
- `protocol`: Protocol object pointer (used to set device protocol)
- `baudRate`: Baud rate (e.g., 115200)

### 3.4 Usage Example

```cpp
#include "device_manager.hpp"
#include "RH56F1_485_protocol.hpp"

// Create device manager
DeviceManager device_manager;

// Create protocol object
auto protocol = std::make_shared<RH56F1_485_Protocol>();
protocol->setDeviceId(1);

// Add device
device_manager.addDevice("/dev/ttyUSB0", protocol, 115200);

// Get device
auto device = device_manager.getDevice("/dev/ttyUSB0");
if (device) {
    // Use device
    std::vector<uint8_t> cmd = {0xEB, 0x90, 0x01, 0x04, 0x11, 0x28, 0x04, 0x0C, 0xXX};
    device->write(cmd);
}

// Remove device
device_manager.removeDevice("/dev/ttyUSB0");
```

### 3.5 Data Flow and Communication

**Device management flow**:
```
addDevice() 
  → Create SerialPortBase object
  → Set protocol
  → Store in devices map
  → Start async receive thread
```

**Device retrieval flow**:
```
getDevice(port) 
  → Look up in devices map
  → Return SerialPortBase object
```

---

## 4. Config Loader (ConfigLoader)

### 4.1 Function

`ConfigLoader` loads device config from YAML config files, creates protocol objects, and configures the logging system.

### 4.2 Main Classes and Functions

```cpp
class ConfigLoader {
public:
    // Load device config
    static std::unordered_map<std::string, DeviceInfo> loadDeviceConfig(const std::string& config_path);
    
    // Create protocol object
    static std::shared_ptr<Protocol> createProtocolFromConfig(const std::string& config_path);
    
    // Configure logging system
    static void configureLogging(const std::string& config_path);
};
```

### 4.3 Config Parameters

**Config file format (device_protocol_config.yaml)**:

```yaml
protocol:
  type: RH56F1_485  # Protocol type

devices:
  - name: hand_left      # Device name
    port: /dev/ttyUSB0   # Serial port
    baudrate: 115200     # Baud rate
    Hand_ID: 1           # Device ID

logging:
  level: DEBUG           # Log level
  file: logs/hand_control.log  # Log file path
  console: true          # Whether to output to console
  file_enable: true      # Whether to output to file
  max_file_size_mb: 10   # Max single log file size (MB)
  max_files: 5           # Number of log files to keep
```

### 4.4 Usage Example

```cpp
#include "config_loader.hpp"
#include "device_manager.hpp"

// Configure logging
ConfigLoader::configureLogging("config/device_protocol_config.yaml");

// Load device config
auto deviceConfig = ConfigLoader::loadDeviceConfig("config/device_protocol_config.yaml");

// Iterate device config
for (const auto& [port, deviceInfo] : deviceConfig) {
    std::cout << "Device: " << deviceInfo.name 
              << ", Port: " << port 
              << ", Baudrate: " << deviceInfo.baudrate 
              << ", Hand_ID: " << deviceInfo.hand_id << std::endl;
}

// Create protocol object
auto protocol = ConfigLoader::createProtocolFromConfig("config/device_protocol_config.yaml");
```

### 4.5 Data Flow and Communication

**Config loading flow**:
```
loadDeviceConfig() 
  → Read YAML file
  → Parse device list
  → Return DeviceInfo map
```

**Protocol creation flow**:
```
createProtocolFromConfig() 
  → Read YAML file
  → Get protocol type
  → ProtocolFactory::create()
  → Return protocol object
```

---

## 5. Logger Manager (LoggerManager)

### 5.1 Function

`LoggerManager` provides unified logging management based on spdlog. Supports console and file output with log rotation and level control.

### 5.2 Main Classes and Functions

```cpp
class LoggerManager {
public:
    // Init logger system (from config struct)
    static void initialize(const LogConfig& config);
    
    // Init logger system (from YAML node)
    static void initialize(const YAML::Node& logging_node);
    
    // Dynamically set log level
    static void setLogLevel(const std::string& level);
    static void setLogLevel(spdlog::level::level_enum level);
    
    // Get log level
    static std::string getLogLevel();
    
    // Get logger object
    static std::shared_ptr<spdlog::logger> getLogger();
    
    // Reconfigure logger system
    static void reconfigure(const LogConfig& config);
    
    // Flush log buffer
    static void flush();
    
    // Shut down logger system
    static void shutdown();
};
```

### 5.3 Config Parameters

**LogConfig struct**:
```cpp
struct LogConfig {
    std::string level = "INFO";              // Log level
    std::string file_path = "logs/app.log";  // Log file path
    bool console_enable = true;              // Output to console
    bool file_enable = true;                 // Output to file
    size_t max_file_size = 10 * 1024 * 1024; // Max single log file size (bytes)
    size_t max_files = 5;                    // Number of log files to keep
};
```

**Log levels**:
- `TRACE`: Most detailed
- `DEBUG`: Debug info
- `INFO`: General info
- `WARN`: Warning
- `ERROR`: Error
- `CRITICAL`: Critical error
- `OFF`: Disable logging

### 5.4 Usage Example

```cpp
#include "logger_manager.hpp"

// Option 1: Init from YAML
YAML::Node config = YAML::LoadFile("config/device_protocol_config.yaml");
LoggerManager::initialize(config["logging"]);

// Option 2: Init from config struct
LoggerManager::LogConfig log_config;
log_config.level = "DEBUG";
log_config.file_path = "logs/app.log";
log_config.console_enable = true;
log_config.file_enable = true;
log_config.max_file_size = 10 * 1024 * 1024;
log_config.max_files = 5;
LoggerManager::initialize(log_config);

// Get logger and use
auto logger = getLogger();
logger->info("This is an info log");
logger->debug("This is a debug log");
logger->warn("This is a warning log");
logger->error("This is an error log");

// Dynamically adjust log level
LoggerManager::setLogLevel("WARN");

// Flush log buffer
LoggerManager::flush();

// Shut down logger system (on exit)
LoggerManager::shutdown();
```

### 5.5 Data Flow and Communication

**Log output flow**:
```
logger->info() 
  → spdlog logger system
  → console sink (if enabled)
  → file sink (if enabled)
  → log file (auto-rotated)
```

**Log rotation**:
- When log file reaches `max_file_size`, a new file is created automatically
- Keeps `max_files` log files; oldest deleted when exceeded

---

## 6. Ring Buffer (RingBuffer)

### 6.1 Function

`RingBuffer` is a ring buffer used to cache serial receive data. Supports efficient enqueue and dequeue operations, supports reading from a specified position without breaking the ring structure.

### 6.2 Main Classes and Functions

```cpp
class RingBuffer {
public:
    explicit RingBuffer(size_t size);
    
    // Clear buffer
    void clear();
    
    // Enqueue: add data to ring buffer
    void push(const uint8_t* data, size_t len);
    
    // Dequeue: read data from buffer
    size_t pop(uint8_t* data, size_t maxlen);
    
    // Get current valid data length in buffer
    size_t size() const;
    
    // Get contiguous data length from tail
    size_t contiguousDataSize() const;
    
    // Get underlying data pointer of buffer
    const uint8_t* data() const;
    
    // Get pointer to contiguous segment from tail
    const uint8_t* dataPtr() const;
    
    // Trim parsed data, move tail pointer
    void advance(size_t count);
    
    // Get underlying buffer data (const ref)
    const std::vector<uint8_t>& getBuffer() const;
    
    // Get tail index
    size_t getTail() const;
};
```

### 6.3 Config Parameters

**Constructor parameter**:
- `size`: Buffer size (e.g., 1024)

### 6.4 Usage Example

```cpp
#include "ring_buffer.hpp"

// Create ring buffer
RingBuffer ringBuffer(1024);

// Add data
uint8_t data[] = {0x90, 0xEB, 0x01, 0x0F, 0x11, 0x28, 0x04, 0x64, 0x00};
ringBuffer.push(data, sizeof(data));

// Get data length
size_t data_size = ringBuffer.size();

// Read data (without breaking ring structure)
uint8_t read_buffer[1024];
size_t read_len = ringBuffer.pop(read_buffer, sizeof(read_buffer));

// Read from specified position (for protocol parsing)
size_t offset = 0;
uint8_t byte = readByteAtOffset(ringBuffer, offset);

// Trim parsed data
ringBuffer.advance(10);  // move tail, discard first 10 bytes

// Clear buffer
ringBuffer.clear();
```

### 6.5 Data Flow and Communication

**Enqueue flow**:
```
push(data, len) 
  → Check buffer space
  → Write data at head position
  → Update head pointer
```

**Dequeue flow**:
```
pop(data, maxlen) 
  → Read data from tail position
  → Update tail pointer
  → Return read length
```

**Trim flow**:
```
advance(count) 
  → Move tail pointer
  → Free space of parsed data
```

---

## 7. Protocol Factory (ProtocolFactory)

### 7.1 Function

`ProtocolFactory` uses factory pattern and a registration mechanism, supporting dynamic registration and creation of protocol objects. New protocols only need to be registered — no need to modify core code.

### 7.2 Main Classes and Functions

```cpp
class ProtocolFactory {
public:
    // Register protocol type
    static void registerProtocol(const std::string& type, ProtocolCreator creator);
    
    // Create protocol object
    static std::shared_ptr<Protocol> create(const std::string& type);
    
    // Get registered protocol type list
    static std::vector<std::string> getRegisteredTypes();
    
    // Check if protocol type is registered
    static bool isRegistered(const std::string& type);
};
```

### 7.3 Config Parameters

**Protocol registration macro**:
```cpp
REGISTER_PROTOCOL("RH56F1_485", RH56F1_485_Protocol);
```

### 7.4 Usage Example

```cpp
#include "protocol_factory.hpp"

// Check if protocol is registered
if (ProtocolFactory::isRegistered("RH56F1_485")) {
    // Create protocol object
    auto protocol = ProtocolFactory::create("RH56F1_485");
    
    // Use protocol
    protocol->setDeviceId(1);
}

// Get all registered protocol types
auto types = ProtocolFactory::getRegisteredTypes();
for (const auto& type : types) {
    std::cout << "Registered protocol: " << type << std::endl;
}
```

### 7.5 Data Flow and Communication

**Protocol registration flow** (auto-executed at program start):
```
REGISTER_PROTOCOL macro
  → Static global object construction
  → Call registerProtocol()
  → Store in registry
```

**Protocol creation flow**:
```
create(type) 
  → Look up creator function in registry
  → Call creator function
  → Return protocol object
```

---

## 8. ROS2 Controller (RegisterController)

### 8.1 Function

`RegisterController` is a ROS2 device control node — one device corresponds to one node. The external ROS type is auto-derived from **`protocol.type`** in **`device_protocol_config.yaml`** as **`RH5DG2` / `RH56F1`**, mapped via **`InterfaceAdapter`** to **`rh5dg2_interfaces` or `rh56f1_interfaces`** (no more unified `RegisterData` / `SetRegister`). Supports topics and services; timer loop reads registers and publishes state.

### 8.2 Main Classes and Functions

```cpp
class RegisterController : public rclcpp::Node {
public:
    RegisterController(
        const std::string& node_name,
        const DeviceNodeConfig& config,
        std::shared_ptr<SerialPortBase> device,
        std::shared_ptr<Protocol> protocol
    );
    
    // Init controller
    void initialize();
    
    // Start control loop
    void start();
    
    // Stop control loop
    void stop();
    
protected:
    // Read register value
    std::pair<bool, std::vector<int>> readRegister(const std::string& reg_name, size_t length = 0);
    
    // Write register value
    bool writeRegister(const std::string& reg_name, const std::vector<int>& values);
    
    // Read touch data
    std::pair<bool, TouchDataResult> readTouchData(int version = 0);
    
    // Control loop (called by timer)
    void controlLoop();
};
```

### 8.3 Config Parameters

**DeviceNodeConfig struct**:
```cpp
struct DeviceNodeConfig {
    std::string device_name;              // Device name
    std::string interfaces_profile;       // Derived from protocol.type (RH5DG2 / RH56F1); don't set in ros2_controller_config.yaml
    std::string publish_frame_id;         // From publish_header.frame_id or frame_id; written to publish msg Header
    std::vector<std::string> joint_names; // Matches model's DOF (RH5DG2:13 / RH56F1:6); written to joint_names array
    std::vector<TopicConfig> topics;      // Topic config list
    std::vector<ServiceConfig> services;  // Service config list
    double update_rate = 50.0;            // Timer update rate (Hz)
};
```

**TopicConfig struct**:
```cpp
struct TopicConfig {
    std::string name;                          // Topic name identifier
    std::vector<std::string> write_registers;  // Write register list
    std::vector<std::string> read_registers;   // Read register list
    std::string command_topic;                 // Command topic name
    std::string state_topic;                   // State topic name
    int touch_version = 1;                     // Touch data version
};
```

**ServiceConfig struct**:
```cpp
struct ServiceConfig {
    std::string register_name;           // Register name
    std::string set_service_name;        // Set service name
    std::string get_service_name;        // Get service name
    bool is_write_register;              // Whether this is a write register
};
```

### 8.4 Usage Example

**Config file (ros2_controller_config.yaml)**:

```yaml
device_nodes:
  - device: hand_left
    # interfaces_profile is auto-derived from protocol.type in device_protocol_config.yaml; don't fill here
    update_rate: 50
    publish_header:
      frame_id: "hand_left"
    joint_names:
      - "hand_left/joint_0"
      # ... 13 items total for RH5DG2; 6 for RH56F1

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

Below are command examples when **`protocol.type`** is RH5DG2 series (for **RH56F1** series, switch to **`rh56f1_interfaces`** and use **6** joints).

**ROS2 topic usage**:

```bash
# Publish angle command (13-dim joint_values)
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

ros2 topic echo /hand_left/angle_actual
```

**ROS2 service usage**:

```bash
ros2 service call /hand_left/set_angle rh5dg2_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

ros2 service call /hand_left/get_errorCode rh5dg2_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"
```

### 8.5 Data Flow and Communication

**Topic mode data flow**:
```
Timer loop (50Hz)
  → controlLoop()
  → readRegister() (read register)
  → publishRegisterData() (publish state topic)
  
Subscriber publishes command
  → Subscriber callback
  → handleCommand()
  → writeRegister() (write register)
```

**Service mode data flow**:
```
Service client call
  → Service callback
  → pauseTimer(3ms) (pause timer)
  → readRegister() / writeRegister()
  → return service response
  → resume timer
```

---

## 9. ROS2 Main Node (inspire_control_node)

### 9.1 Function

`inspire_control_node` is the ROS2 main entry point. Responsible for initializing ROS2, loading config, creating devices and controllers, and running the ROS2 event loop.

### 9.2 Main Functions

1. **Init ROS2**: `rclcpp::init()`
2. **Load config files**: device config and ROS2 controller config
3. **Configure logger**: load logging config from config file
4. **Create device manager**: manages all serial port devices
5. **Create protocol objects**: independent protocol instance per device
6. **Create controller nodes**: a RegisterController node per device
7. **Run event loop**: `MultiThreadedExecutor::spin_some()`

### 9.3 Config Parameters

**Command-line arguments**:
- `--device-config`: Device protocol config file path
- `--controller-config`: ROS2 controller config file path
- `--device`: Single-device mode, specify device name

### 9.4 Usage Example

**Build ROS2 workspace**:

```bash
cd /home/ubuntu/serial_control/src/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2
source install/setup.bash
```

**Launch single-device node**:

```bash
# Via launch file
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py \
  device_name:=hand_left

# Or run node directly
ros2 run inspire_control_ros2 inspire_control_node \
  --device-config /path/to/device_protocol_config.yaml \
  --controller-config /path/to/ros2_controller_config.yaml \
  --device hand_left
```

**Launch multi-device node**:

```bash
# Via launch file
ros2 launch inspire_control_ros2 inspire_control_multi_device.launch.py

# Or run node directly (no --device argument)
ros2 run inspire_control_ros2 inspire_control_node \
  --device-config /path/to/device_protocol_config.yaml \
  --controller-config /path/to/ros2_controller_config.yaml
```

**View node info**:

```bash
# List nodes
ros2 node list

# List topics
ros2 topic list

# List services
ros2 service list

# Show node info
ros2 node info /hand_left_node
```

### 9.5 Data Flow and Communication

**Startup flow**:
```
Program start
  → Init ROS2
  → Load config files
  → Configure logger
  → Create device manager
  → Create protocol objects
  → Create controller nodes
  → Start timer loop
  → Run ROS2 event loop
```

**Runtime flow**:
```
ROS2 event loop
  ├─ Timer callback (50Hz)
  │  └─ controlLoop() (read register and publish)
  ├─ Topic subscriber callback
  │  └─ handleCommand() (write register)
  └─ Service callback
     └─ Dedicated .srv per register (created by adapter, types in rh*_interfaces)
```

---

## 10. Common Usage Scenarios

### 10.1 Single Device Control

```bash
# Start left hand device node
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py \
  device_name:=hand_left

# Publish angle command (RH5DG2: 13-dim)
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

# Subscribe to angle state
ros2 topic echo /hand_left/angle_actual
```

### 10.2 Multi-device Parallel Control

```bash
# Start all device nodes
ros2 launch inspire_control_ros2 inspire_control_multi_device.launch.py

# Control left and right hands simultaneously
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

ros2 topic pub --once /hand_right/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"
```

### 10.3 Service Calls

```bash
# Set device comm ID
ros2 service call /hand_left/set_id rh5dg2_interfaces/srv/Setid \
  "{hand_id: 1, device_id: 2}"

# Get error code
ros2 service call /hand_left/get_errorCode rh5dg2_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"

# Clear error (fields per Setclearerror.srv)
ros2 service call /hand_left/set_clearError rh5dg2_interfaces/srv/Setclearerror \
  "{hand_id: 1, clear_code: 1}"
```

### 10.4 Read Touch Data

```bash
# Subscribe touch data topic
ros2 topic echo /hand_left/touch_data
```

---

## 11. Debug and Troubleshooting

### 11.1 View Logs

```bash
# View log file
tail -f logs/hand_control.log

# View ROS2 logs
ros2 run inspire_control_ros2 inspire_control_node --ros-args --log-level debug
```

### 11.2 Check Serial Port

```bash
# List serial devices
ls -l /dev/ttyUSB*

# Check serial permissions
sudo chmod 666 /dev/ttyUSB0

# Test serial communication
sudo minicom -D /dev/ttyUSB0 -b 115200
```

### 11.3 Check ROS2 Nodes

```bash
# List nodes
ros2 node list

# List topics
ros2 topic list

# List services
ros2 service list

# View topic data
ros2 topic echo /hand_left/angle_actual

# Check service type
ros2 service type /hand_left/set_angle
ros2 service type /hand_left/get_errorCode
```

---

**Doc version**: v1.1 (aligned with strongly-typed interface packages and inspire_control_ros2)  
**Last updated**: 2026-05-12
