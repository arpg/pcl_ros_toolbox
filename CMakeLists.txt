cmake_minimum_required(VERSION 2.8)
project(pcl_ros_toolbox)

set(PACKAGE_DEPENDENCIES
  roscpp
  pcl_ros
  pcl_conversions
  std_msgs
  std_srvs
  nodelet
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
)

add_library(approx_time_concatenate_server src/approx_time_concatenate_server.cpp)
target_link_libraries(approx_time_concatenate_server ${LINK_LIBS})

add_library(mixed_concatenate_server src/mixed_concatenate_server.cpp)
target_link_libraries(mixed_concatenate_server ${LINK_LIBS})
