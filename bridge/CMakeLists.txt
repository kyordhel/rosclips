cmake_minimum_required(VERSION 3.14)
project(bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB BRIDGE_SRC
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

## Declare static library
add_library(bridge STATIC
  ${BRIDGE_SRC}
)

target_include_directories(bridge
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(bridge
  clips64
  clipswrapper
)

ament_target_dependencies(bridge
  rclcpp
  std_msgs
)

# set_target_properties(bridge
#   PROPERTIES
#   OUTPUT_NAME "lib${PROJECT_NAME}.a"
#   PREFIX ""
#   SUFFIX ""
# )
