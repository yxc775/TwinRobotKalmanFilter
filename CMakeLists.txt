cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(modelR1_control SHARED modelR1_control.cc)
target_link_libraries(modelR1_control ${GAZEBO_LIBRARIES})

add_library(modelR2_control SHARED modelR2_control.cc)
target_link_libraries(modelR2_control ${GAZEBO_LIBRARIES})

add_library(Beacon SHARED Beacon.cc)
target_link_libraries(Beacon
  ${GAZEBO_LIBRARIES}
)
