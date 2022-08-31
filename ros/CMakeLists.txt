cmake_minimum_required(VERSION 3.22)

project(mc_udp_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mc_udp_ros_server ros_server.cpp)
target_link_directories(mc_udp_ros_server PUBLIC "/home/lian/mc_udp/build/src")
target_link_directories(mc_udp_ros_server PUBLIC "/home/lian/mc_udp/build/src/server")
target_include_directories(mc_udp_ros_server PUBLIC "/home/lian/mc_udp/include")
target_link_libraries(mc_udp_ros_server ${catkin_LIBRARIES} libmc_udp_data.so libmc_udp_server.so)