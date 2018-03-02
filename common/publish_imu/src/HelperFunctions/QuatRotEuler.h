#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include <math.h>
#include <Eigen/Dense>

//Function to help setting data into a ROS Vector3
geometry_msgs::Vector3 SetVector3(float x, float y, float z);

//Function to set quaternion values
geometry_msgs::Quaternion setQuat(float qx, float qy, float qz, float qw);

//Function to convert a rotation matrix to quaternion
geometry_msgs::Quaternion rot2quat(Eigen::Matrix3d R);

//Function to calculate the quaternion multiplication
geometry_msgs::Quaternion quatProd(geometry_msgs::Quaternion q1,
	                               geometry_msgs::Quaternion q2);

//Function to calculate the conjugate (inverse) of a quaternion
geometry_msgs::Quaternion quatInv(geometry_msgs::Quaternion quat);

//Function to get yaw from a quaternion
double getHeadingFromQuat(geometry_msgs::Quaternion quat);

//Function to convert quaternion to Roll-Pitch_yaw
geometry_msgs::Vector3 quat2rpy(geometry_msgs::Quaternion quat);

//Function to convert Roll-Pitch-yaw to quaternion
geometry_msgs::Quaternion rpy2quat(geometry_msgs::Vector3 rpy);

//Rotation matrix for a rotation about the x-axis
Eigen::Matrix3d rotx(double theta);

//Rotation matrix for a rotation about the y-axis
Eigen::Matrix3d roty(double theta);

//Rotation matrix for a rotation about the z-axis
Eigen::Matrix3d rotz(double theta);