cmake_minimum_required(VERSION 3.8)
project(odrive_wheels_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# -- Standalone C++ driver library (no ROS dependencies) --
add_library(odrive_wheels_driver_core STATIC
  src/can_socket.cpp
  src/odrive_wheels_driver/odrive_driver.cpp
)

target_include_directories(odrive_wheels_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# -- ROS 2 wrapper node --
add_executable(odrive_wheels_driver_node
  src/odrive_wheels_driver_node.cpp
)

target_include_directories(odrive_wheels_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(odrive_wheels_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_link_libraries(odrive_wheels_driver_node odrive_wheels_driver_core)

install(TARGETS odrive_wheels_driver_core
  ARCHIVE DESTINATION lib
)

install(TARGETS odrive_wheels_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_keyboard
)

install(PROGRAMS
  scripts/check_odrive_config.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_odrive_config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_latest_event_mailbox
    test/test_latest_event_mailbox.cpp
  )
  target_include_directories(test_latest_event_mailbox PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
