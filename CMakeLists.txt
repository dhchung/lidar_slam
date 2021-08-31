cmake_minimum_required(VERSION 3.0.2)
project(lidar_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)
find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})



file(GLOB SOURCES
     ${PROJECT_SOURCE_DIR}/include/lidar_slam/*.cpp
     ${PROJECT_SOURCE_DIR}/include/lidar_slam/*.h)

catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_msgs pcl_ros roscpp sensor_msgs std_msgs tf
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include/lidar_slam
    ${Eigen_INCLUDE_DIRS}


)


add_executable(${PROJECT_NAME}_node
               src/main.cpp
               ${SOURCES}
               ${Eigen_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)
