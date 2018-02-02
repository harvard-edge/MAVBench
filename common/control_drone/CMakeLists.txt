cmake_minimum_required(VERSION 2.8.3)
project(control_drone)
set(darknet_include_dir  $ENV{HOME}/darknet/include $ENV{HOME}/darknet/src $ENV{HOME})
set(darknet_lib_dir $ENV{HOME}/darknet/obj)
set(darknet_lib_dir $ENV{HOME}/darknet/obj)
set (common_utility_src $ENV{HOME}/catkin_ws/src/mav-bench/common)
set (common_utility_include $ENV{HOME}/catkin_ws/src/mav-bench/common)


#-- collecting all the files in the darknet_obj folder
#file(GLOB darknet_obj_files
#    "*.o"
#)

set (Airlib_addr  $ENV{HOME}/AirSim/AirLib)
set (CUDA_OPENCVLIBS
opencv_cudaoptflow3
opencv_cudawarping3
opencv_cudaimgproc3
opencv_cudaobjdetect3
opencv_cudacodec3
opencv_cudaoptflow3
opencv_cudastereo3
opencv_cudacodec3
opencv_cudalegacy3
opencv_cudawarping3
opencv_cudabgsegm3
opencv_cudafilters3
opencv_cudafilters3
opencv_cudaarithm3
opencv_cudabgsegm3
opencv_cudastereo3
opencv_cudalegacy3
opencv_cudalegacy3
opencv_cudafeatures2d3
opencv_cudaimgproc3
opencv_cudacodec3
opencv_cudaimgproc3
opencv_cudafilters3
opencv_cudafeatures2d3
opencv_cudabgsegm3
opencv_cudaobjdetect3
opencv_cudaobjdetect3
opencv_cudaarithm3
opencv_cudaoptflow3
opencv_cudawarping3
opencv_cudafeatures2d3
opencv_cudaarithm3
opencv_cudastereo3)

find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
message(darknet_include  ${CUDA_LIBRARIES})
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

#message(cuda_include_dirs )
include_directories(
    ${Eigen_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    $ENV{HOME}/AirSim/AirLib/include
    ${common_utility_include} 
    ${darknet_include_dir}
    ${CUDA_INCLUDE_DIRS}
)

link_directories(
  $ENV{HOME}/AirSim/AirLib/lib
  $ENV{HOME}/AirSim/AirLib/deps/rpclib/lib
  $ENV{HOME}/AirSim/AirLib/deps/rpclib/lib
  /usr/local/cuda/lib64  #added by Behzad (for Behzad)
  ${Airlib_addr}/deps/MavLinkCom/lib  
  /usr/lib 
  /opt/ros/kinetic/lib
  ${darknet_lib_dir}  
  $ENV{HOME}/darknet
)

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS trajectory_msgs mav_msgs tf message_runtime
)



add_executable(control_thread ${common_utility_src}/control_drone_node.cpp ${common_utility_src}/control_drone.cpp
   ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/objdetect.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(control_thread ${CUDA_OPENCVLIBS}  ${catkin_LIBRARIES}  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} darknet AirLib MavLinkCom rpc pthread)
add_dependencies(control_thread ${catkin_EXPORTED_TARGETS})




#
#


#
#
