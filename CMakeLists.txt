cmake_minimum_required(VERSION 2.8.0)
project(orb_slam_2_ros)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  image_geometry
  image_transport
  )

catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)

set(ORB_SLAM_PATH ~/work/px4/ORB_SLAM2)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${ORB_SLAM_PATH}
  ${ORB_SLAM_PATH}/include
)


cs_add_library(orb_slam_2_interface
  src/library/logger.cpp
  src/library/interface.cpp
  src/library/interface_mono.cpp
  src/library/interface_stereo.cpp
  src/library/interface_rgbd.cpp
)

cs_add_executable(orb_slam_2_ros_node
  src/orb_slam_2_ros_node.cpp
)



set(ORB_SLAM_LIBS
  ${ORB_SLAM_PATH}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM_PATH}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM_PATH}/lib/libORB_SLAM2.so
  )

target_link_libraries(orb_slam_2_ros_node orb_slam_2_interface ${catkin_LIBRARIES} ${ORB_SLAM_LIBS})

cs_install()
cs_export()
