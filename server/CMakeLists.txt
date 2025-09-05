cmake_minimum_required(VERSION 3.14)
set(PARENT_PROJECT_NAME "${PROJECT_NAME}")
project(server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB ROSCLIPS_SRC
  ${PROJECT_SOURCE_DIR}/*.cpp
)

## Declare an executable
add_executable(server
  ${ROSCLIPS_SRC}
)

target_include_directories(server
  PUBLIC
)

target_link_libraries(server
  clips64
  clipswrapper
  bridge
)

ament_target_dependencies(server
  rclcpp
  std_msgs
)

rosidl_target_interfaces(server
  ${PARENT_PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
