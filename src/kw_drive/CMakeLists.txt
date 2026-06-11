cmake_minimum_required(VERSION 3.16)
project(kw_drive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(kw_motor
  src/protocol/kw_motor.cpp
  src/protocol/kw_serial_transport.cpp
)
target_include_directories(kw_motor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(kw_motor
  PUBLIC
    Threads::Threads
)

add_executable(kw_motor_test
  src/kw_motor_test_node.cpp
)
ament_target_dependencies(kw_motor_test
  rclcpp
  sensor_msgs
)
target_link_libraries(kw_motor_test
  kw_motor
)

add_executable(kw_smart_knob
  src/kw_smart_knob_node.cpp
)
ament_target_dependencies(kw_smart_knob
  rclcpp
  sensor_msgs
)
target_link_libraries(kw_smart_knob
  kw_motor
)

install(TARGETS
  kw_motor
  kw_motor_test
  kw_smart_knob
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
