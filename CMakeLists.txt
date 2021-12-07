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
  mesh_msgs
  goicp
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  include
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

add_library(stamped_mesh_to_pcl src/stamped_mesh_to_pcl.cpp)
target_link_libraries(stamped_mesh_to_pcl ${LINK_LIBS})

add_library(pcl_aggregator src/pcl_aggregator.cpp)
target_link_libraries(pcl_aggregator ${LINK_LIBS} ${Boost_LIBRARIES})

add_library(pcl_reframer src/pcl_reframer.cpp)
target_link_libraries(pcl_reframer ${LINK_LIBS})

add_library(pcl_transformer src/pcl_transformer.cpp)
target_link_libraries(pcl_transformer ${LINK_LIBS})

add_executable(pcd_publisher src/pcd_publisher.cpp)
target_link_libraries(pcd_publisher ${LINK_LIBS})

add_library(groundtruth_trajectory_generator src/groundtruth_trajectory_generator.cpp)
target_link_libraries(groundtruth_trajectory_generator ${LINK_LIBS})

add_executable(groundtruth_trajectory_generator_node src/groundtruth_trajectory_generator_node.cpp)
target_link_libraries(groundtruth_trajectory_generator_node groundtruth_trajectory_generator)
