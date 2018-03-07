cmake_minimum_required(VERSION 2.8.3)
project(publish_imu)
set (common_utility_src $ENV{HOME}/catkin_ws/src/mav-bench/common)
set (common_utility_include $ENV{HOME}/catkin_ws/src/mav-bench/common)


set (Airlib_addr  $ENV{HOME}/AirSim/AirLib)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  mav_msgs
  tf
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  cv_bridge
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

#message(cuda_include_dirs )
include_directories(
    ${Eigen_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    $ENV{HOME}/AirSim/AirLib/include
    ${common_utility_include} 
    ${CUDA_INCLUDE_DIRS}
    ${common_utility_src}/publish_imu/src
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
  CATKIN_DEPENDS mav_msgs tf message_runtime
)


add_executable(publish_imu ${common_utility_src}/publish_imu/src/publish_imu.cpp 
    ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/publish_imu/src/HelperFunctions/QuatRotEuler.cpp) 
target_link_libraries(publish_imu ${catkin_LIBRARIES}  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} AirLib MavLinkCom rpc pthread)
add_dependencies(publish_imu ${catkin_EXPORTED_TARGETS})



