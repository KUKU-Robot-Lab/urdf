cmake_minimum_required(VERSION 3.10)
project(inspire_hand_serial_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# 头文件目录
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# 源文件
set(SOURCES
    src/serial_port.cpp
    src/device_manager.cpp
    src/ring_buffer.cpp
    src/RH56F1_485_protocol.cpp
    src/RH5DG2_485_protocol.cpp
    src/RH56F1_canfd_protocol.cpp
    src/RH5DG2_canfd_protocol.cpp
    src/config_loader.cpp
    src/protocol_factory.cpp  # 协议工厂实现
    src/logger_manager.cpp    # 日志管理器实现
)

# 可执行文件
add_executable(serial_hand_control_node 
    examples/main.cpp 
    ${SOURCES}
)

# 链接库
target_link_libraries(serial_hand_control_node 
    ${Boost_LIBRARIES}
    yaml-cpp
    pthread
    spdlog::spdlog
)
