cmake_minimum_required(VERSION 3.16)
project(damiao_arm_sdk VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(damiao_arm_sdk STATIC
    src/can_transport.cpp
    src/damiao_usb_transport.cpp
    src/motor_driver.cpp
)

target_include_directories(damiao_arm_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

find_package(Threads REQUIRED)
target_link_libraries(damiao_arm_sdk PUBLIC Threads::Threads)

# Single-motor demo executable
add_executable(single_motor_demo examples/single_motor_demo.cpp)
target_link_libraries(single_motor_demo PRIVATE damiao_arm_sdk)

# SocketCAN <-> Damiao USB-CAN bridge daemon
add_executable(damiao_socketcan_bridge examples/damiao_socketcan_bridge.cpp)
target_link_libraries(damiao_socketcan_bridge PRIVATE damiao_arm_sdk)

# Unit & vcan0 integration tests
enable_testing()
add_executable(test_sdk tests/test_sdk.cpp)
target_link_libraries(test_sdk PRIVATE damiao_arm_sdk)
add_test(NAME DamiaoSdkV01Tests COMMAND test_sdk)
