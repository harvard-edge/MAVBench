#include "ros/ros.h"
#include "mavbench/get_trajectory.h"
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// *** F:DN simply, sends and arm command

bool send_trajectory(mavbench::get_trajectory::Request  &req,
         mavbench::get_trajectory::Response &res)
{
   

  // *** F:DN hardcoding the trajectory
  geometry_msgs::Vector3 my_vec;
  my_vec.x = 0;
  my_vec.y = 0;
  my_vec.z = 2;
  
  geometry_msgs::Twist my_twist;
  my_twist.linear = my_vec;
  trajectory_msgs::MultiDOFJointTrajectoryPoint point;
  point.velocities.push_back(my_twist);
  res.multiDOFtrajectory.points.push_back(point);
  
  my_vec.x = 0;
  my_vec.y = 0;
  my_vec.z = -2;
  my_twist.linear = my_vec;
  trajectory_msgs::MultiDOFJointTrajectoryPoint point_2;
  point_2.velocities.push_back(my_twist);
  res.multiDOFtrajectory.points.push_back(point_2);


  ROS_INFO("sending back response\n");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Drone_dummy_service_adv_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("dummy_service_adv", send_trajectory);
  ROS_INFO("Ready to send trajectories.");
  ros::spin();

  return 0;
}
