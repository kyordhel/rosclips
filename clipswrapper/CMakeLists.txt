cmake_minimum_required(VERSION 3.14)
project(clipswrapper)

file(GLOB WRAPPER_SRC
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

## Declare static library
add_library(clipswrapper STATIC
  ${WRAPPER_SRC}
)

target_include_directories(clipswrapper
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include/clipswrapper
)

target_include_directories(clipswrapper
  PUBLIC
  # ${CLIPS64_HEADERS}
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(clipswrapper
  clips64
)

# set_target_properties(clipswrapper
#   PROPERTIES
#   OUTPUT_NAME "lib${PROJECT_NAME}.a"
#   PREFIX ""
#   SUFFIX ""
# )
