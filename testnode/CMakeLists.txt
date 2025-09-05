cmake_minimum_required(VERSION 3.14)
project(testnode)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

## Declare an executable
add_executable(testnode
  main.cpp
)

# target_link_libraries(testnode
#   pthread
# )

ament_target_dependencies(testnode
  rclcpp
  std_msgs
)