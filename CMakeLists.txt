cmake_minimum_required(VERSION 2.8.3)
project(tiburon_simulator)



# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  synchronizer
  thruster_controller
)



# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(tiburon_simulator src/buoyancy_plugin/buoyancy.cc include/tiburon_simulator/buoyancy/buoyancy.hh)
target_link_libraries(tiburon_simulator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(imu src/imu_plugin/imu.cc)
target_link_libraries(imu ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(thruster_plugin src/thruster_plugin/thruster.cpp)
target_link_libraries(thruster_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
