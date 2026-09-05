# Dexterous Hand Control Project — Architecture Overview

## 1. Project Overview

A ROS2-based dexterous hand control system that communicates with multiple dexterous hand devices via RS485 serial. Modular design supporting multi-device parallel control, with both Topic and Service communication modes.

### 1.1 Core Features

- ✅ **Multi-device support**: Simultaneously controls multiple dexterous hand devices (e.g., left hand, right hand)
- ✅ **Multi-protocol support**: Supports multiple communication protocols (RH56F1_485, RH5DG2_485, etc.) via factory pattern
- ✅ **Dynamic configuration**: Flexible device and ROS2 interface configuration via YAML files
- ✅ **Dual communication modes**: Topic (real-time control) and Service (on-demand call)
- ✅ **Asynchronous serial communication**: Boost.Asio-based async serial with timeout and error handling
- ✅ **Unified logging system**: Global logger manager with file rotation and level control
- ✅ **Thread-safe**: Safe communication in multi-threaded environment

---

## 2. System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        User Interface Layer                          │
├─────────────────────────────────────────────────────────────────────┤
│  ROS2 topic subscribers   ROS2 service clients    Debug/example     │
│  (publish ctrl cmds)      (call service interface) (main.cpp)        │
└────────────────────┬──────────────────┬─────────────────────────────┘
                     │                  │
                     ▼                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        ROS2 Interface Layer                          │
├─────────────────────────────────────────────────────────────────────┤
│  RegisterController (device node)                                    │
│  ├─ Topic interface                                                  │
│  │   ├─ Publisher: publish state (/hand_left/angle_actual)           │
│  │   └─ Subscriber: subscribe ctrl cmd (/hand_left/angle_set)        │
│  │                                                                   │
│  └─ Service interface                                                │
│      └─ Each register has its own .srv (rh5dg2_interfaces / rh56f1_interfaces) │
│                                                                       │
│  Timer loop (50Hz)                                                   │
│  └─ controlLoop(): periodic register read and state publish          │
└────────────────────┬─────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        Protocol Abstraction Layer                    │
├─────────────────────────────────────────────────────────────────────┤
│  Protocol (abstract base class)                                      │
│  ├─ buildReadCommand(): build read command                           │
│  ├─ buildWriteCommand(): build write command                         │
│  ├─ parseResponse(): parse response data                             │
│  ├─ readRegister(): read register                                    │
│  └─ writeRegister(): write register                                  │
│                                                                       │
│  Concrete protocol implementations:                                  │
│  ├─ RH56F1_485_Protocol (RH56F1 protocol)                            │
│  └─ RH5DG2_485_Protocol (RH5DG2 protocol)                            │
│                                                                       │
│  Protocol factory:                                                   │
│  └─ ProtocolFactory: dynamically create protocol objects             │
└────────────────────┬─────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        Device Management Layer                       │
├─────────────────────────────────────────────────────────────────────┤
│  DeviceManager                                                       │
│  └─ Manages multiple serial devices (port -> SerialPortBase map)     │
│                                                                       │
│  SerialPortBase (serial abstract base class)                         │
│  ├─ write(): blocking write                                          │
│  ├─ read(): blocking read (with timeout)                             │
│  └─ Async receive thread (Boost.Asio)                                │
│                                                                       │
│  RingBuffer                                                          │
│  └─ Buffers serial receive data, supports frame parsing              │
└────────────────────┬─────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        Hardware Interface Layer                      │
├─────────────────────────────────────────────────────────────────────┤
│  485 serial communication                                            │
│  ├─ /dev/ttyUSB0 (left hand device)                                  │
│  └─ /dev/ttyUSB1 (right hand device)                                 │
│                                                                       │
│  Dexterous hand hardware                                             │
│  ├─ RH56F1 series                                                    │
│  └─ RH5DG2 series                                                    │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                        Support Systems                               │
├─────────────────────────────────────────────────────────────────────┤
│  Config system:                                                      │
│  ├─ ConfigLoader: load YAML config files                             │
│  ├─ device_protocol_config.yaml: device protocol config              │
│  └─ ros2_controller_config.yaml: ROS2 controller config              │
│                                                                       │
│  Logging system:                                                     │
│  └─ LoggerManager: global logger manager (spdlog)                    │
│      ├─ Console output                                               │
│      ├─ File output (rotated)                                        │
│      └─ Log level control                                            │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. Data Flow Diagrams

### 3.1 Topic Mode Data Flow (Real-time Control)

```
┌──────────────────┐
│ ROS2 topic       │
│ subscriber       │
│ (user program)   │
└──────┬───────────┘
       │ publish ctrl cmd
       │ /hand_left/angle_set
       ▼
┌─────────────────────────────────────┐
│ RegisterController                  │
│ ├─ Subscriber callback              │
│ │  └─ handleCommand()               │
│ │      └─ writeRegister()           │
│ │          └─ Protocol::writeRegister()│
│ │              └─ buildWriteCommand()│
│ │                  └─ SerialPort::write()│
│ │                      └─ 485 serial │
│ │                          └─ hardware│
│ │                                      │
│ └─ Timer loop (50Hz)                  │
│     └─ controlLoop()                  │
│         └─ readRegister()            │
│             └─ Protocol::readRegister()│
│                 └─ buildReadCommand() │
│                     └─ SerialPort::read()│
│                         └─ 485 serial │
│                             └─ hardware│
│                                 │      │
│                                 │ response│
│                                 ▼      │
│                          parseResponse()│
│                                 │      │
│                                 ▼      │
│                          publishRegisterData()│
│                                 │      │
│                                 ▼      │
└─────────────────────────────────────────┘
       │ publish state data
       │ /hand_left/angle_actual
       ▼
┌──────────────────┐
│ ROS2 topic       │
│ subscriber       │
│ (user program)   │
└──────────────────┘
```

### 3.2 Service Mode Data Flow (On-demand Call)

```
┌──────────────────┐
│ ROS2 service     │
│ client           │
│ (user program)   │
└──────┬───────────┘
       │ call service
       │ /hand_left/set_angle
       ▼
┌─────────────────────────────────────┐
│ RegisterController                  │
│ ├─ Service callback                 │
│ │  └─ Dedicated srv callback → writeRegister│
│ │      ├─ pauseTimer(3ms)            │
│ │      └─ writeRegister()            │
│ │          └─ Protocol::writeRegister()│
│ │              └─ buildWriteCommand()│
│ │                  └─ SerialPort::write()│
│ │                      └─ 485 serial │
│ │                          └─ hardware│
│ │                                      │
│ │ Response handling                    │
│ │  └─ parseResponse()                  │
│ │      └─ Return service response      │
│ │          └─ success + values         │
└─────────────────────────────────────┘
       │ return service response
       │ (success, values)
       ▼
┌──────────────────┐
│ ROS2 service     │
│ client           │
│ (user program)   │
└──────────────────┘
```

---

## 4. Module Relationship Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    inspire_control_node.cpp                  │
│                    (ROS2 main entry)                         │
│  ├─ Init ROS2                                                │
│  ├─ Load config files                                        │
│  ├─ Create DeviceManager                                     │
│  ├─ Create Protocol objects                                  │
│  └─ Create RegisterController nodes                          │
└────────────┬────────────────────────────────────────────────┘
             │
             ├─────────────────────────────────────┐
             │                                     │
             ▼                                     ▼
┌────────────────────────┐          ┌────────────────────────┐
│   ConfigLoader         │          │   DeviceManager         │
│   (config loader)      │          │   (device manager)      │
│                        │          │                        │
│  ├─ loadDeviceConfig() │          │  ├─ addDevice()        │
│  ├─ createProtocol()   │─────────▶│  ├─ getDevice()        │
│  └─ configureLogging() │          │  └─ removeDevice()     │
└────────────────────────┘          └────────────┬───────────┘
             │                                   │
             │                                   │
             ▼                                   ▼
┌────────────────────────┐          ┌────────────────────────┐
│   ProtocolFactory      │          │   SerialPortBase       │
│   (protocol factory)   │          │   (serial comm)        │
│                        │          │                        │
│  ├─ create()           │          │  ├─ write()            │
│  └─ registerProtocol() │          │  ├─ read()             │
│                        │          │  └─ Async recv thread  │
└────────────┬───────────┘          └────────────────────────┘
             │
             │ creates
             ▼
┌────────────────────────┐
│   Protocol             │
│   (abstract base)      │
│                        │
│  ├─ RH56F1_485         │
│  └─ RH5DG2_485         │
│                        │
│  Uses:                 │
│  ├─ SerialPortBase     │
│  └─ RingBuffer         │
└────────────────────────┘

┌────────────────────────┐
│   RegisterController   │
│   (ROS2 device node)   │
│                        │
│  Uses:                 │
│  ├─ Protocol           │
│  ├─ SerialPortBase     │
│  └─ RingBuffer         │
│                        │
│  Provides:             │
│  ├─ Topic interface    │
│  └─ Service interface  │
└────────────────────────┘

┌────────────────────────┐
│   LoggerManager        │
│   (logger manager)     │
│                        │
│  └─ Global singleton   │
│     └─ spdlog          │
└────────────────────────┘
```

---

## 5. Key Components

### 5.1 Hardware Interface Layer

**Function**: Physical communication with dexterous hand hardware

**Components**:
- **485 serial**: Connect device via USB-to-485 adapter
- **Serial devices**: `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.

**Characteristics**:
- Multi-device parallel communication
- Each device has independent serial port and protocol instance
- Async receive, blocking send

---

### 5.2 Device Management Layer

**Function**: Manage serial port devices and protocol objects

**Components**:
- **SerialPortBase**: Serial abstract base class
  - Boost.Asio-based async communication
  - Timeout read support
  - Thread-safe receive buffer
- **DeviceManager**: Device manager
  - Manages multiple serial port devices
  - Port-to-device mapping
- **RingBuffer**: Ring buffer
  - Buffers serial receive data
  - Supports frame parsing and data extraction

---

### 5.3 Protocol Abstraction Layer

**Function**: Implements concrete communication protocols

**Components**:
- **Protocol**: Protocol abstract base class
  - Defines unified protocol interface
  - Command building and response parsing
- **Concrete protocol implementations**:
  - `RH56F1_485_Protocol`: RH56F1 series protocol
  - `RH5DG2_485_Protocol`: RH5DG2 series protocol
- **ProtocolFactory**: Protocol factory
  - Dynamically create protocol objects
  - Supports protocol registration and extension

**Characteristics**:
- Factory pattern, easy to extend with new protocols
- Each device has independent protocol instance
- Supports dynamic read length

---

### 5.4 ROS2 Interface Layer

**Function**: Provides ROS2 topic and service interfaces

**Components**:
- **RegisterController**: Device control node
  - One node per device
  - Supports multiple topics and services
  - Timer loop reads and publishes

**Communication modes**:
1. **Topic mode**:
   - Subscribe to control command topic
   - Periodically publish state topic
   - High real-time, suitable for continuous control

2. **Service mode**:
   - Provides dedicated service types per register (defined in the model's interface package)
   - On-demand call, not part of timer loop
   - Suitable for single operations and queries

---

### 5.5 Configuration System

**Function**: Load config from YAML files

**Config files**:
1. **device_protocol_config.yaml**:
   - Device config (port, baudrate, Hand_ID)
   - Protocol type config
   - Logging config

2. **ros2_controller_config.yaml**:
   - ROS2 node config
   - Topic config (command topic, state topic)
   - Service config (Set/Get services)

**Components**:
- **ConfigLoader**: Config loader
  - Parses YAML files
  - Creates protocol objects
  - Configures logging system

---

### 5.6 Logging System

**Function**: Unified logging management

**Components**:
- **LoggerManager**: Logger manager
  - Global singleton
  - Based on spdlog
  - Supports console and file output
  - Supports log rotation

**Characteristics**:
- Thread-safe
- Supports dynamic log level adjustment
- Files auto-rotate (by size and count)

---

## 6. Threading Model

```
Main thread (inspire_control_node)
│
├─ ROS2 event loop thread (MultiThreadedExecutor)
│  └─ RegisterController node
│     ├─ Timer thread (50Hz)
│     │  └─ controlLoop(): read register and publish
│     ├─ Topic subscriber callback thread
│     │  └─ handleCommand(): write register
│     └─ Service callback thread
│        └─ InterfaceAdapter + rh*_interfaces: bind topic and service types
│
└─ Serial async receive thread (one per device)
   └─ SerialPortBase::io_thread_
      └─ Boost.Asio async read
         └─ Data stored in receive_buffer_
```

**Thread-safety mechanisms**:
- Serial port writes use mutex protection
- Serial port reads use condition variable sync
- Timer pause mechanism prevents service/timer conflict
- RingBuffer supports multi-threaded access

---

## 7. Startup Flow

```
1. Program start (inspire_control_node)
   │
   ├─ 2. Init ROS2 (rclcpp::init)
   │
   ├─ 3. Load config files
   │   ├─ device_protocol_config.yaml
   │   └─ ros2_controller_config.yaml
   │
   ├─ 4. Configure logging (LoggerManager::initialize)
   │
   ├─ 5. Create DeviceManager
   │
   ├─ 6. Load device config (ConfigLoader::loadDeviceConfig)
   │   └─ Parse device list (port, baudrate, Hand_ID)
   │
   ├─ 7. Create protocol object per device
   │   ├─ ProtocolFactory::create()
   │   └─ protocol->setDeviceId(Hand_ID)
   │
   ├─ 8. Add devices to DeviceManager
   │   └─ Create SerialPortBase object
   │       └─ Start async receive thread
   │
   ├─ 9. Create RegisterController nodes
   │   ├─ Load ROS2 config
   │   ├─ Create Topic interface (Publisher/Subscriber)
   │   ├─ Create Service interface
   │   └─ Create timer (50Hz)
   │
   ├─ 10. Start all controllers (controller->start())
   │
   └─ 11. Run ROS2 event loop
       └─ MultiThreadedExecutor::spin_some()
```

---

## 8. Extension Points

### 8.1 Add New Protocol

1. Create new protocol class inheriting `Protocol`
2. Implement all pure virtual functions
3. Register with `REGISTER_PROTOCOL` macro
4. Specify protocol type in config file

### 8.2 Add New Register

1. Add register address in `REGISTER_MAP` of the protocol class
2. Set read length in `REGISTER_READ_LENGTH_MAP` (if needed)
3. Add topic or service config in ROS2 config file

### 8.3 Add New Device

1. Add device config in `device_protocol_config.yaml`
2. Add device node config in `ros2_controller_config.yaml`
3. System will auto-recognize and start it

---

## 9. Performance Characteristics

- **Control frequency**: 50Hz (20ms period)
- **Serial communication**: Async receive, blocking send
- **Timeout mechanism**: 25ms read timeout
- **Multi-device support**: Each device has independent thread and protocol instance
- **Memory management**: Smart pointers, auto resource management

---

## 10. Fault Handling

- **Serial communication errors**: Timeout retry, log recorded
- **Protocol parse errors**: Skip invalid frame, continue processing
- **Service call exceptions**: try-catch protection, ensure response returned
- **Timer conflict**: Pause timer 3ms during service call

---

**Doc version**: v1.0
**Last updated**: 2026-05-12
