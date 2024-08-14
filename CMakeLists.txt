# CMake 프로그램의 최소 버전
cmake_minimum_required(VERSION 3.11)

# 프로젝트 정보
project(
  ModooCode
  VERSION 0.1
  DESCRIPTION "예제 프로젝트"
  LANGUAGES CXX
)

# add_library(shape STATIC ${SRC_FILES})
set (SOURCES
  src/main.cpp
  src/kinematics.cpp
  src/controller.cpp
  src/filter.cpp
  src/trajectory.cpp
)

add_executable (main ${SOURCES})

target_include_directories(main PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/../../include
#  /usr/include/eigen3
)

target_link_directories(main PUBLIC       #Library directory
    ${CMAKE_SOURCE_DIR}/../../lib
)

target_link_libraries(main
    mujoco
    glfw
    dl
)


