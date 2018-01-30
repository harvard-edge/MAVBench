#include "Drone.h"
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <cstring>
#include "common/Common.hpp"
#include "coord.h"
#include <geometry_msgs/Point.h>
#include "timer.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include "common/VectorMath.hpp"
Drone::Drone() : client(0)
{
	connect();
    //initial_gps = {0, 0, 0};
    //initial_gps = gps();
    this->localization_method = "ground_truth";
}

Drone::Drone(const std::string& ip_addr, uint16_t port) : client(0), collision_count(0), 
    localization_method("ground_truth")
{
	connect(ip_addr, port);
    //initial_gps = {0, 0, 0};
    //initial_gps = gps();
}

Drone::Drone(const std::string& ip_addr, uint16_t port, std::string localization_method) : client(0), collision_count(0)
{
	connect(ip_addr, port);
    //initial_gps = {0, 0, 0};
    //initial_gps = gps();
    this->localization_method = localization_method;
}

Drone::~Drone()
{
	if (client != 0)
		delete client;
}

void Drone::connect()
{
	if (client != 0)
		delete client;
	client = new msr::airlib::MultirotorRpcLibClient();
    client->enableApiControl(true);
}

void Drone::set_localization_method(std::string localization_method) {
    this->localization_method = localization_method;
}
void Drone::connect(const std::string& ip_addr, uint16_t port)
{
	if (client != 0)
		delete client;
	client = new msr::airlib::MultirotorRpcLibClient(ip_addr, port);
    client->enableApiControl(true);
}

void Drone::arm()
{
	client->armDisarm(true);
}

void Drone::disarm()
{
	client->armDisarm(false);
}

bool Drone::takeoff(double h)
{
    auto pos = position();
    const double margin = 0.2;

    while (pos.z < h-margin || pos.z > h+margin) {
        for (; pos.z < h-margin || pos.z > h+margin; pos = position()) {
            const float p = 0.75, max_speed = 1;
            float dist = h - pos.z;

            float speed = dist*p;
            if (speed > max_speed)
                speed = max_speed;

            fly_velocity(0, 0, speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        fly_velocity(0,0,0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    return true;
}

bool Drone::set_yaw(int y)
{
    int pos_dist = (y - int(get_yaw()) + 360) % 360;
    int yaw_diff = pos_dist <= 180 ? pos_dist : pos_dist - 360;

    float duration = yaw_diff / max_yaw_rate;
    if (duration < 0)
        duration = -duration;

    float yaw_rate = max_yaw_rate;
    if (yaw_diff < 0)
        yaw_rate = -yaw_rate;

	try {
        auto drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        auto yawmode = msr::airlib::YawMode(true, yaw_rate);

        client->moveByVelocity(0, 0, 0, duration, drivetrain, yawmode);

        int duration_ms = duration*1000;
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
	}catch(...){
		std::cerr << "set_yaw failed" << std::endl;
		return false;
	}
}

/*
bool Drone::set_yaw(float y, bool slow)
{
    float angular_vel = 15;

	try {
        if (slow) {
            float init_yaw = get_yaw();
            int direction;
            if (y >= init_yaw)
                direction = 1;
            else
                direction = -1;

            angular_vel *= direction;

            for (float yaw = get_yaw(); (direction == 1 && yaw <= y) || (direction == -1 && yaw >= y);) {
                // ROS_ERROR("yaw: %f, y: %f", yaw, y);

                client->rotateToYaw(yaw, 60, 5);

                yaw += angular_vel;

                if (direction == 1 && yaw > y)
                    yaw = y+1;
                else if (direction == -1 && yaw < y)
                    yaw = y-1;

                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        } else {
            client->rotateToYaw(y, 60, 5);
        }
	}catch(...){
		std::cerr << "set_yaw failed" << std::endl;
		return false;
	}

	return true;
}
*/

bool Drone::set_yaw_based_on_quaternion(geometry_msgs::Quaternion q)
{
	float pitch, roll, yaw;
    Eigen::Quaternion<float,Eigen::DontAlign> q_airsim_style;
    q_airsim_style.x() = q.x;
    q_airsim_style.y() = q.y;
    q_airsim_style.z() = q.z;
    q_airsim_style.w() = q.w;
    msr::airlib::VectorMath::toEulerianAngle(q_airsim_style, pitch, roll, yaw);
	
    try {
        this->set_yaw(yaw*180 / M_PI);
    } catch(...) {
		std::cerr << "set_yaw_based_on_quaternion failed" << std::endl;
		return false;
	}
	return true;
}

static float xy_yaw(double x, double y) {
    float angle_to_dest;

    if (x == 0 && y == 0)
        angle_to_dest = 0;
    else if (x == 0) {
        angle_to_dest = y > 0 ? 0 : -180;
    } else if (y == 0) {
        angle_to_dest = x > 0 ? 90 : -90;
    } else if (x > 0 && y > 0) {
        angle_to_dest = std::atan(x/y) * 180.0/3.14;
    } else if (x > 0 && y < 0) {
        angle_to_dest = 180.0 - (std::atan(-x/y) * 180.0/3.14);
    } else if (x < 0 && y > 0) {
        angle_to_dest = -(std::atan(-x/y) * 180.0/3.14);
    } else if (x < 0 && y < 0) {
        angle_to_dest = -180 + (std::atan(x/y) * 180.0/3.14);
    }

    return angle_to_dest;
}

bool Drone::fly_velocity(double vx, double vy, double vz, float yaw, double duration)
{
    getCollisionInfo();

	try {
        if (yaw != YAW_UNCHANGED) {
            float target_yaw = yaw != FACE_FORWARD ? yaw : xy_yaw(vx, vy);
            float yaw_diff = (int(target_yaw - get_yaw()) + 360) % 360;
            yaw_diff = yaw_diff <= 180 ? yaw_diff : yaw_diff - 360;
            float yaw_rate = yaw_diff / duration;

            if (yaw_rate > max_yaw_rate_during_flight)
                yaw_rate = max_yaw_rate_during_flight;

            auto drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
            auto yawmode = msr::airlib::YawMode(true, yaw_rate);

            client->moveByVelocity(vy, vx, -vz, duration, drivetrain, yawmode);
        } else {
            client->moveByVelocity(vy, vx, -vz, duration);
        }
    } catch(...) {
		std::cerr << "fly_velocity failed" << std::endl;
		return false;
	}

	return true;
}

bool Drone::land()
{
    auto pos = position();

    for (; pos.z > 0.05; pos = position()) {
        const float speed = -0.5;
        fly_velocity(0, 0, speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    fly_velocity(0,0,0);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    return true;
}
/*
coord Drone::gps()
{
	getCollisionInfo();
	auto pos = client->getPosition();
 
    coord curr_pos = {pos.y(), pos.x(), -pos.z()};
    coord result = {curr_pos.x - initial_gps.x,
                    curr_pos.y - initial_gps.y,
                    curr_pos.z - initial_gps.z};
    
    return result;
}
*/

geometry_msgs::Pose Drone::pose()
{
    geometry_msgs::Pose result;
    /*
    if (this->localization_method == "gps") {
        auto p = client->getPosition();
	    auto q = client->getOrientation();
        result.position.x = client->getPosition().y() - initial_gps.x;
        result.position.y = client->getPosition().x() - initial_gps.y;
        result.position.z = -1*client->getPosition().z() - initial_gps.z;
        result.orientation.x = q.x();
        result.orientation.y = q.y();
        result.orientation.z = q.z();
        result.orientation.w = q.w();
    }else{
     */
    //std::cout<<"localization method is"<<this->localization_method<<std::endl;
    tf::StampedTransform transform;
        try{
            tfListen.lookupTransform("/world", "/"+(this->localization_method),
                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return result;
        }

        tf::Vector3 tf_translation = transform.getOrigin();
        result.position.x = tf_translation.x();
        result.position.y = tf_translation.y();
        result.position.z = tf_translation.z();

        tf::Quaternion tf_rotation = transform.getRotation();
        result.orientation.x = tf_rotation.x();
        result.orientation.y = tf_rotation.y();
        result.orientation.z = tf_rotation.z();
        result.orientation.w = tf_rotation.w();
    //}
    return result;
}

coord Drone::position() {
    coord result;
    geometry_msgs::Pose result_pose = pose();
    result.x = result_pose.position.x;
    result.y = result_pose.position.y;
    result.z = result_pose.position.z;
    return result;
}


geometry_msgs::PoseWithCovariance Drone::pose_with_covariance()
{
    geometry_msgs::PoseWithCovariance result;

    result.pose = pose();

    for (int i = 0; i <36; i++) { 
        //https://answers.ros.org/question/181689/computing-posewithcovariances-6x6-matrix/ 
        result.covariance[i] = 0;
    }

    return result;
}

float Drone::get_yaw()
{
	//auto q = client->getOrientation();
	auto pose = this->pose();
    msr::airlib::Quaternionr q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return -y*180 / M_PI;
}

float Drone::get_roll()
{
	//auto q = client->getOrientation();
	auto pose = this->pose();
   msr::airlib::Quaternionr q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    
    float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return r*180 / M_PI;
}

/*
geometry_msgs::Pose Drone::get_geometry_pose(){
    geometry_msgs::Pose pose;
	auto q = client->getOrientation();
	//auto p = client->getPosition();
    pose.position.x = client->getPosition().y();
    pose.position.y = client->getPosition().x();
    pose.position.z = -1*client->getPosition().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}
*/
/*
geometry_msgs::PoseWithCovariance Drone::get_geometry_pose_with_coveraiance(){
    geometry_msgs::PoseWithCovariance pose_with_covariance;
    geometry_msgs::Pose pose;
	
    auto q = client->getOrientation();
    pose.position.x = client->getPosition().y();
    pose.position.y = client->getPosition().x();
    pose.position.z = -1*client->getPosition().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_with_covariance.pose = pose;
    for (int i = 0; i <36; i++) { 
        //https://answers.ros.org/question/181689/computing-posewithcovariances-6x6-matrix/ 
        pose_with_covariance.covariance[i] = 0;
    } 
    return pose_with_covariance;
}
*/

float Drone::get_pitch()
{
	
    auto pose = this->pose();
    msr::airlib::Quaternionr q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
     //auto q = client->getOrientation();
	
    
    float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return p*180 / M_PI;
}

msr::airlib::CollisionInfo Drone::getCollisionInfo()
{
  auto col_info = client->getCollisionInfo();
  if (col_info.has_collided) {
     collision_count ++;
  }
  if (collision_count > 10) {
    land();
    sleep(5);
    disarm();
    fprintf(stderr, "Drone Crashed!\n");
    LOG_TIME(crashed);
    ros::shutdown();
    exit(0);
  }

  if (col_info.has_collided) {
  printf("CollisionInfo %s, count %ld, collison %d, penetrate %f\n",
          col_info.has_collided ? "has collided" : "none", collision_count, col_info.collison_count,
          col_info.penetration_depth);
  }

  return col_info;
}
