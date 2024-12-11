cmake_minimum_required(VERSION 3.10)
project(camera)

# Find DepthAI package
find_package(depthai CONFIG REQUIRED)

# Include directories (optional if find_package works correctly)
include_directories(${depthai_INCLUDE_DIRS})

# Add your executable
add_executable(camera camera.cpp)

# Link DepthAI libraries
target_link_libraries(camera PRIVATE depthai::opencv)
