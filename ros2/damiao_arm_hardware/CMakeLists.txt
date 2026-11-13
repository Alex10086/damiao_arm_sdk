cmake_minimum_required(VERSION 3.22)
project(damiao_arm_hardware VERSION 0.2.0 LANGUAGES CXX)

if(CMAKE_CXX_STANDARD LESS 17)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

set(DAMIAO_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(OPENARM_CAN_ROOT "${DAMIAO_SDK_ROOT}/third_party/openarm_can")

add_library(damiao_arm_sdk_embedded STATIC
  ${DAMIAO_SDK_ROOT}/src/arm.cpp
  ${DAMIAO_SDK_ROOT}/src/damiao_usb_transport.cpp
  ${DAMIAO_SDK_ROOT}/src/socketcan_bridge.cpp
  ${OPENARM_CAN_ROOT}/src/openarm/can/socket/arm_component.cpp
  ${OPENARM_CAN_ROOT}/src/openarm/can/socket/gripper_component.cpp
  ${OPENARM_CAN_ROOT}/src/openarm/can/socket/openarm.cpp
  ${OPENARM_CAN_ROOT}/src/openarm/canbus/can_device_collection.cpp
  ${OPENARM_CAN_ROOT}/src/openarm/canbus/can_socket.cpp
  ${OPENARM_CAN_ROOT}/src/openarm/damiao_motor/dm_motor.cpp
  ${OPENARM_CAN_ROOT}/src/openarm/damiao_motor/dm_motor_control.cpp
  ${OPENARM_CAN_ROOT}/src/openarm/damiao_motor/dm_motor_device.cpp
  ${OPENARM_CAN_ROOT}/src/openarm/damiao_motor/dm_motor_device_collection.cpp
)
set_target_properties(damiao_arm_sdk_embedded PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(damiao_arm_sdk_embedded
  PUBLIC
    $<BUILD_INTERFACE:${DAMIAO_SDK_ROOT}/include>
    $<BUILD_INTERFACE:${OPENARM_CAN_ROOT}/include>
)
target_link_libraries(damiao_arm_sdk_embedded PUBLIC Threads::Threads)

add_library(${PROJECT_NAME} SHARED
  src/damiao_system_hardware.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(${PROJECT_NAME}
  damiao_arm_sdk_embedded
)

pluginlib_export_plugin_description_file(hardware_interface damiao_arm_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
