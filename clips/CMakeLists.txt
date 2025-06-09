cmake_minimum_required(VERSION 3.14)

project(clips)
add_definitions(-w) # disable all warnings

file(GLOB CLIPS_SRC
  ${PROJECT_SOURCE_DIR}/src/*.c
)

## Declare a C library
add_library(clips64 STATIC
  ${CLIPS_SRC}
)

target_include_directories(clips64
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include/clips
)

# set(CLIPS64_HEADERS ${PROJECT_SOURCE_DIR}/include TOP_LEVEL)
target_include_directories(clips64
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

# set_target_properties(clips64
#   PROPERTIES
#   OUTPUT_NAME "lib${PROJECT_NAME}.so"
#   PREFIX ""
#   SUFFIX ""
# )
