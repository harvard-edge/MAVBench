cmake_minimum_required(VERSION 2.8.3)
project(timed_roslaunch)

find_package(catkin REQUIRED COMPONENTS)

#add_message_files(
#    FILES
#    )


#add_service_files(
#  FILES
#)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

#message(cuda_include_dirs )
include_directories(
    ${catkin_INCLUDE_DIRS}
)

link_directories(
  /usr/lib 
  /opt/ros/kinetic/lib
)

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS
)

