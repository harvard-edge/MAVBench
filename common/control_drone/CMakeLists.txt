cmake_minimum_required(VERSION 2.8.3)
project(control_drone)
set (common_utility_src $ENV{HOME}/catkin_ws/src/mav-bench/common)
set (common_utility_include $ENV{HOME}/catkin_ws/src/mav-bench/common)

set (Airlib_addr  $ENV{HOME}/AirSim/AirLib)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  multiagent_collision_check
  trajectory_msgs
  mav_msgs
  tf
  cv_bridge
  image_transport
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf
  message_generation
  nlopt
  )

#add_message_files(
#    FILES
#    )


#add_service_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
    ${Eigen_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    $ENV{HOME}/AirSim/AirLib/include
    ${common_utility_include} 
)

link_directories(
  $ENV{HOME}/AirSim/AirLib/lib
  $ENV{HOME}/AirSim/AirLib/deps/rpclib/lib
  $ENV{HOME}/AirSim/AirLib/deps/rpclib/lib
  /usr/local/cuda/lib64  #added by Behzad (for Behzad)
  ${Airlib_addr}/deps/MavLinkCom/lib  
  /usr/lib 
  /opt/ros/kinetic/lib
)

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS trajectory_msgs mav_msgs tf message_runtime
)



add_executable(control_thread ${common_utility_src}/control_drone_node.cpp ${common_utility_src}/control_drone.cpp
   ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(control_thread ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread)
add_dependencies(control_thread ${catkin_EXPORTED_TARGETS})




#
#


#
#
