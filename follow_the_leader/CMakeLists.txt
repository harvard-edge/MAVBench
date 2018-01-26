cmake_minimum_required(VERSION 2.8.3)
project(follow_the_leader)
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

add_message_files(
    FILES
    bounding_box_msg.msg 
    OD.msg
    )


add_service_files(
  FILES
  cmd_srv.srv
  shut_down.srv
  )

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
    $ENV{HOME}/catkin_ws/src/mav-bench/follow_the_leader
    $ENV{HOME}/catkin_ws/src/mav-bench/follow_the_leader/src/kcf
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


#add_executable(control_drone ${common_utility_src}/control_drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/timer.cpp)
#target_link_libraries(control_drone ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread )

#add_executable(follow_the_leader src/main.cpp src/misc.cpp src/error.cpp src/log__class.cpp src/objdetect.cpp src/track.cpp src/pid.cpp kcf/kcftracker.cpp kcf/fhog.cpp ${common_utility_src}/Drone.cpp)
#target_link_libraries(follow_the_leader ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread )
#
#add_executable(follow_trajectory src/follow_trajectory.cpp ${common_utility_src}/common.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/timer.cpp)
#target_link_libraries(follow_trajectory ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread)
#
#add_executable(SAR src/SAR.cpp  ${common_utility_src}/Drone.cpp src/control_drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/timer.cpp)
#target_link_libraries(SAR ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread ${CUDA_OPENCVLIBS})
#add_dependencies(SAR ${catkin_EXPORTED_TARGETS})
#
#add_executable(exploration src/exploration.cpp src/control_drone.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/timer.cpp)
#target_link_libraries(exploration ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread ${CUDA_OPENCVLIBS})
#add_dependencies(exploration ${catkin_EXPORTED_TARGETS})
#
#add_executable(object_detection_node src/object_detection_node.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/objdetect.cpp ${common_utility_src}/timer.cpp)
#target_link_libraries(object_detection_node ${CUDA_OPENCVLIBS}  ${catkin_LIBRARIES}  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} darknet AirLib MavLinkCom rpc pthread)
#add_dependencies(object_detection_node ${catkin_EXPORTED_TARGETS})

add_executable(detection_node src/detection_thread.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/objdetect.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(detection_node ${CUDA_OPENCVLIBS}  ${catkin_LIBRARIES}  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} darknet AirLib MavLinkCom rpc pthread)
add_dependencies(detection_node ${catkin_EXPORTED_TARGETS})


add_executable(buffer_track_node src/buffer_track.cpp src/track.cpp src/kcf/kcftracker.cpp src/kcf/fhog.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/objdetect.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(buffer_track_node ${CUDA_OPENCVLIBS}  ${catkin_LIBRARIES}  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} darknet AirLib MavLinkCom rpc pthread)
add_dependencies(buffer_track_node ${catkin_EXPORTED_TARGETS})


add_executable(tracking_continuous src/tracking_continuous.cpp src/track.cpp src/kcf/kcftracker.cpp src/kcf/fhog.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/objdetect.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(tracking_continuous ${CUDA_OPENCVLIBS}  ${catkin_LIBRARIES}  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} darknet AirLib MavLinkCom rpc pthread)
add_dependencies(tracking_continuous ${catkin_EXPORTED_TARGETS})





add_executable(main_follow_the_leader_node src/main_follow_the_leader.cpp ${common_utility_src}/control_drone.cpp
   ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/objdetect.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(main_follow_the_leader_node ${CUDA_OPENCVLIBS}  ${catkin_LIBRARIES}  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} darknet AirLib MavLinkCom rpc pthread)
add_dependencies(main_follow_the_leader_node ${catkin_EXPORTED_TARGETS})


add_executable(pid_node src/pid_thread.cpp src/pid.cpp ${common_utility_src}/control_drone.cpp
    ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/objdetect.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(pid_node ${CUDA_OPENCVLIBS}  ${catkin_LIBRARIES}  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} darknet AirLib MavLinkCom rpc pthread)
add_dependencies(pid_node ${catkin_EXPORTED_TARGETS})

add_executable(dummy_node src/dummy.cpp ${common_utility_src}/Drone.cpp ${common_utility_src}/common.cpp ${common_utility_src}/objdetect.cpp ${common_utility_src}/timer.cpp)
target_link_libraries(dummy_node ${CUDA_OPENCVLIBS}  ${catkin_LIBRARIES}  ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} darknet AirLib MavLinkCom rpc pthread)
add_dependencies(dummy_node ${catkin_EXPORTED_TARGETS})







#
#


#
#
