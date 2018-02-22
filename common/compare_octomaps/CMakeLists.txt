cmake_minimum_required(VERSION 2.8.3)
project(compare_octomaps)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  octomap_world
  kdtree
  rviz_visual_tools
  )
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

#add_service_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#  visualization_msgs
#)

catkin_package(
    #include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
    #LIBRARIES ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
    #CATKIN_DEPENDS octomap_world
    #message_runtime roscpp geometry_msgs visualization_msgs octomap_world tf 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  /opt/ros/kinetic/include
  )

#add_library(nbvPlannerLib src/mesh_structure.cpp src/nbvp.cpp src/rrt.cpp src/tree.cpp)
#add_executable(nbvPlanner src/nbv_planner_node.cpp src/mesh_structure.cpp src/nbvp.cpp src/rrt.cpp src/tree.cpp)
add_executable(compare_octomaps src/compare_octomaps.cpp)
add_dependencies(compare_octomaps ${PROJECT_NAME}_generate_msgs)

target_link_libraries(compare_octomaps
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

