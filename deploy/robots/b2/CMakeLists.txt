cmake_minimum_required(VERSION 3.12)
project(b2_controller)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

# Use the installed Unitree SDK package and allow an env override for custom installs.
if(DEFINED ENV{UNITREE_SDK2_PATH})
    set(UNITREE_SDK2_PATH $ENV{UNITREE_SDK2_PATH})
    if(EXISTS "${UNITREE_SDK2_PATH}/lib/cmake/unitree_sdk2")
        list(APPEND CMAKE_PREFIX_PATH "${UNITREE_SDK2_PATH}")
    endif()
endif()
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")

find_package(unitree_sdk2 REQUIRED CONFIG)

set(UNITREE_SDK2_RUNTIME_DIR "/opt/unitree_robotics/lib")
if(DEFINED UNITREE_SDK2_PATH AND EXISTS "${UNITREE_SDK2_PATH}/lib")
    set(UNITREE_SDK2_RUNTIME_DIR "${UNITREE_SDK2_PATH}/lib")
endif()
set(CMAKE_BUILD_RPATH "${UNITREE_SDK2_RUNTIME_DIR}")
set(CMAKE_INSTALL_RPATH "${UNITREE_SDK2_RUNTIME_DIR}")

include_directories(
  /usr/include/eigen3
  include
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_libraries(
  unitree_sdk2 rt pthread
  Boost::program_options yaml-cpp fmt
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so
)

file(GLOB_RECURSE ADD_SRC_LIST
  src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(b2_ctrl main.cpp)
