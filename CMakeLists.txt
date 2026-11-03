cmake_minimum_required(VERSION 3.22)
project(damiao_arm_sdk VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# Build OpenArm CAN submodule (unmodified)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static openarm_can for SDK integration" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Skip openarm_can internal tests" FORCE)
add_subdirectory(third_party/openarm_can EXCLUDE_FROM_ALL)

add_library(damiao_arm_sdk STATIC
    src/arm.cpp
    src/damiao_usb_transport.cpp
    src/socketcan_bridge.cpp
)

target_include_directories(damiao_arm_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

find_package(Threads REQUIRED)
target_link_libraries(damiao_arm_sdk PUBLIC openarm_can Threads::Threads)

# SocketCAN <-> Damiao USB-CAN bridge daemon
add_executable(damiao_socketcan_bridge examples/damiao_socketcan_bridge.cpp)
target_link_libraries(damiao_socketcan_bridge PRIVATE damiao_arm_sdk)

# Phase 1 OpenArm CAN + Bridge single-motor hardware & communication test
add_executable(openarm_single_motor_test examples/openarm_single_motor_test.cpp)
target_link_libraries(openarm_single_motor_test PRIVATE damiao_arm_sdk)

# Unit & vcan0/pty integration tests
enable_testing()
add_executable(test_sdk tests/test_sdk.cpp)
target_link_libraries(test_sdk PRIVATE damiao_arm_sdk util)
add_test(NAME DamiaoSdkTests COMMAND test_sdk)

add_executable(test_arm tests/test_arm.cpp)
target_compile_definitions(test_arm PRIVATE DAMIAO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(test_arm PRIVATE damiao_arm_sdk)
add_test(NAME DamiaoArmTests COMMAND test_arm)
