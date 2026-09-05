cmake_minimum_required(VERSION 3.10)
project(inspire_control_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rh5dg2_interfaces REQUIRED)
find_package(rh56f1_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(spdlog REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/include
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

set(ROS2_SOURCES
    src/register_controller.cpp
    src/interface_adapter_factory.cpp
    src/RH5DG2_interface_adapter.cpp
    src/RH56F1_interface_adapter.cpp
    src/inspire_controller_factory.cpp
    src/inspire_control_node.cpp
)

set(EXISTING_SOURCES
    ../../../src/config_loader.cpp
    ../../../src/device_manager.cpp
    ../../../src/serial_port.cpp
    ../../../src/ring_buffer.cpp
    ../../../src/RH56F1_485_protocol.cpp
    ../../../src/RH5DG2_485_protocol.cpp
    ../../../src/RH56F1_canfd_protocol.cpp
    ../../../src/RH5DG2_canfd_protocol.cpp
    ../../../src/protocol_factory.cpp
    ../../../src/logger_manager.cpp
)

add_executable(inspire_control_node
    ${ROS2_SOURCES}
    ${EXISTING_SOURCES}
)

ament_target_dependencies(inspire_control_node
    rclcpp
    std_msgs
    rh5dg2_interfaces
    rh56f1_interfaces
)

target_link_libraries(inspire_control_node
    ${Boost_LIBRARIES}
    yaml-cpp
    pthread
    spdlog::spdlog
)

install(TARGETS inspire_control_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
