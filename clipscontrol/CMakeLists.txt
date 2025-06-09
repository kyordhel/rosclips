cmake_minimum_required(VERSION 3.0.2)
set(PARENT_PROJECT_NAME "${PROJECT_NAME}")
project(clipscontrol)

set(CURSES_NEED_NCURSES TRUE)
set(CURSES_NEED_WIDE TRUE)
find_package(Curses REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
# add_definitions(-w) # disable all warnings

file(GLOB CLIPSCONTROL_SRC
  ${PROJECT_SOURCE_DIR}/*.cpp
)

## Declare an executable
add_executable(clipscontrol
  ${CLIPSCONTROL_SRC}
)

target_include_directories(clipscontrol
  PUBLIC
  ${CURSES_INCLUDE_DIR}
)

target_link_libraries(clipscontrol
  Boost::filesystem
  ${BOOST_LIBRARIES}
  ${CURSES_LIBRARIES}
)

ament_target_dependencies(clipscontrol
  rclcpp
  std_msgs
)

rosidl_target_interfaces(clipscontrol
  ${PARENT_PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

