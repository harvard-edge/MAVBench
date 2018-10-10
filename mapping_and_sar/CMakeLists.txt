cmake_minimum_required(VERSION 2.8.3)

project(mapping_and_sar)
set(darknet_include_dir  $ENV{darknet_base_dir}/include $ENV{darknet_base_dir}/src $ENV{darknet_base_dir})
set(darknet_lib_dir $ENV{darknet_base_dir}/obj)
set (common_utility_src $ENV{base_dir}/catkin_ws/src/mav-bench/common)
set (common_utility_include $ENV{base_dir}/catkin_ws/src/mav-bench/common)

#-- collecting all the files in the darknet_obj folder
#file(GLOB darknet_obj_files
#    "*.o"
#)

set (Airlib_addr  $ENV{AirSim_base_dir}/AirLib)
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
  octomap_server
  pcl_conversions
  pcl_ros
  tf
  message_generation
  nlopt
  future_collision
)

add_message_files(
    FILES
    OD.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


#message(cuda_include_dirs )
include_directories(
    ${CATKIN_DEVEL_PREFIX}/include
    ${Eigen_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    $ENV{AirSim_base_dir}/AirLib/include
    $ENV{mavbench_apps_base_dir}/follow_the_leader/
    ${common_utility_include} 
    ${darknet_include_dir}
    ${CUDA_INCLUDE_DIRS}
)

link_directories(
  $ENV{AirSim_base_dir}/AirLib/lib
  $ENV{AirSim_base_dir}/AirLib/deps/rpclib/lib
  $ENV{AirSim_base_dir}/AirLib/deps/rpclib/lib
  /usr/local/cuda/lib64  #added by Behzad (for Behzad)
  ${Airlib_addr}/deps/MavLinkCom/lib  
  /usr/lib 
  /opt/ros/kinetic/lib
  /opt/ros/kinetic/lib/aarch64-linux-gnu
  ${darknet_lib_dir}  
  $ENV{darknet_base_dir}
)

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS trajectory_msgs mav_msgs tf message_runtime
)


#add_executable(control_drone_node ${common_utility_src}/control_drone_node.cpp ${common_utility_src}/control_drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/timer.cpp)
#target_link_libraries(control_drone_node ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread )

add_executable(publish_pose src/publish_pose.cpp ${common_utility_src}/common.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(publish_pose ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread )

# add_executable(follow_trajectory src/follow_trajectory.cpp ${common_utility_src}/common.cpp ${common_utility_src}/Profiling.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/timer.cpp)
# target_link_libraries(follow_trajectory ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread)

add_executable(sar src/sar.cpp  ${common_utility_src}/Drone.cpp ${common_utility_src}/control_drone.cpp ${common_utility_src}/common.cpp  ${common_utility_src}/Profiling.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(sar ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread ${CUDA_OPENCVLIBS})
add_dependencies(sar ${catkin_EXPORTED_TARGETS} mapping_and_sar_generate_messages_cpp)

add_executable(mapping src/mapping.cpp ${common_utility_src}/control_drone.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp  ${common_utility_src}/Profiling.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread)
add_dependencies(mapping ${catkin_EXPORTED_TARGETS})

# add_executable(future_collision src/future_collision ${common_utility_src}/Drone.cpp ${common_utility_src}/timer.cpp)
# target_link_libraries(future_collision ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread)

add_executable(object_detection_node src/object_detection_node.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/objdetect.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(object_detection_node  ${CUDA_OPENCVLIBS} ${catkin_LIBRARIES} ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} darknet AirLib MavLinkCom rpc pthread)
add_dependencies(object_detection_node ${catkin_EXPORTED_TARGETS} mapping_and_sar_generate_messages_cpp)

add_executable(occupancy_map_node src/occupancy_map_node.cpp ${common_utility_src}/common.cpp ${common_utility_src}/Profiling.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/timer.cpp)
add_dependencies(occupancy_map_node future_collision mapping_and_sar_generate_messages_cpp)
target_link_libraries(occupancy_map_node ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread cuda cudart cublas curand )


