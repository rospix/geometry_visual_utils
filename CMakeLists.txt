cmake_minimum_required(VERSION 2.8.3)
project(geometry_visual_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

include(GNUInstallDirs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  image_transport
  camera_info_manager
  cv_bridge
  rostest
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(Threads REQUIRED)
#find_package(sitl_gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen roscpp gazebo_ros
  LIBRARIES geometry_visual_utils geometry_utils
  )

include_directories(
  include/${PROJECT_NAME}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIR}
  )

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(
  geometry_visual_utils
  src/visual_utils.cpp
  )

add_library(
  geometry_utils
  src/geometry_utils.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
