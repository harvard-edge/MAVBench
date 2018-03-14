#include "ros/ros.h"

// Standard headers
#include <math.h>
#include <stdio.h>
#include <signal.h>

// PointCloud headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// ROS message headers
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"

// MAVBench headers
#include "common.h"
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int g_panic_distance;

//profiling variables
ros::Time start_hook_t, end_hook_t;                                          
long long g_panic_kernel_acc = 0;
long long g_panic_main_loop = 0;
int g_panic_ctr;

void log_data_before_shutting_down(){

    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    profiling_data_srv_inst.request.key = "panic_kernel";
    profiling_data_srv_inst.request.value = (((double)g_panic_kernel_acc)/1e9)/g_panic_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "panic_main_loop";
    profiling_data_srv_inst.request.value = (((double)g_panic_main_loop)/1e9)/g_panic_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
}

void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        ros::shutdown();
    }
    exit(0);
}

template<class ObstacleData>
class ObstacleSensor {
public:
    
    ObstacleSensor(ros::NodeHandle& nh, const std::string* obstacle_data_topics)
    {
        sub_1 = nh.subscribe<ObstacleData>(obstacle_data_topics[0], 1, &ObstacleSensor::callback_front, this);
        sub_2 = nh.subscribe<ObstacleData>(obstacle_data_topics[1], 1, &ObstacleSensor::callback_back, this);

        panic_back = false;
        panic_front = false;
    
        for (int i = 0; i++; i<4) {
            this->quadrant_front[i] = 0;
            this->quadrant_back[i] = 0;
        }
        panic_distance = g_panic_distance;
    }

    void callback_front(const typename ObstacleData::ConstPtr& msg)
    {
        start_hook_t = ros::Time::now();

        pcl::PointXYZ closest(0,0,0);
        //closest_pt_front.x = 0; 
        //closest_pt_front.y = 0; 
        //closest_pt_front.z = 0; 
        float min_distance = 10000;
        for (int i = 0; i<4; i++) {
            this->quadrant_front[i] = 0;
        }
        panic_front = false;
        for (const pcl::PointXYZ& pt : msg->points) {
            if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
                if (distance(pt.x, pt.y, pt.z) < min_distance){
                    min_distance = distance(pt.x, pt.y, pt.z);
                    closest.x = pt.x;
                    closest.y = pt.y;
                    closest.z = pt.z;
                }
            }
        }

        if (min_distance < panic_distance){
            panic_front = true;
            if (closest.x > 0 && closest.z > 0)
                quadrant_front[0]++;
            if (closest.x < 0 && closest.z > 0)
                quadrant_front[1]++;
            if (closest.x < 0 && closest.z < 0)
                quadrant_front[2]++;
            if (closest.x > 0 && closest.z < 0)
                quadrant_front[3]++;
            closest_pt_front = closest; 
        } 

        end_hook_t = ros::Time::now();
        g_panic_kernel_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
        g_panic_ctr++;
    }

    void callback_back(const typename ObstacleData::ConstPtr& msg)
    {
        start_hook_t = ros::Time::now();
        pcl::PointXYZ closest(0,0,0);
        //closest_pt_back.x = 0; 
        //closest_pt_back.y = 0; 
        //closest_pt_back.z = 0; 
        float min_distance = 10000;
        panic_back = false;
        for (int i = 0; i<4; i++) {
            this->quadrant_back[i] = 0;
        }

        for (const pcl::PointXYZ& pt : msg->points) {
            if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
                if (distance(pt.x, pt.y, pt.z) < min_distance){
                    min_distance = distance(pt.x, pt.y, pt.z);
                    closest.x = pt.x;
                    closest.y = pt.y;
                    closest.z = pt.z;
                }
            }
        }
        
        if (min_distance < panic_distance){
            //ROS_INFO_STREAM("min dis"<<min_distance);
            panic_back = true;
            if (closest.x > 0 && closest.z > 0)
                quadrant_back[0]++;
            if (closest.x < 0 && closest.z > 0)
                quadrant_back[1]++;
            if (closest.x < 0 && closest.z < 0)
                quadrant_back[2]++;
            if (closest.x > 0 && closest.z < 0)
                quadrant_back[3]++;
            closest_pt_back = closest; 
        }
        
        end_hook_t = ros::Time::now();
        g_panic_kernel_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
        g_panic_ctr++;
    }

    std_msgs::Bool get_panic_data(){
        std_msgs::Bool panic_msg;
        panic_msg.data = panic_front || panic_back;
        return panic_msg;
    }

    geometry_msgs::Vector3 safe_flight_velocity()//const int quadrant_front[4], const int quadrant_back[4], pcl::PointXYZ& closest_pt_front, pcl::PointXYZ& closest_pt_back)
    {
        geometry_msgs::Vector3 velocity;
        geometry_msgs::Vector3 direction;
        direction.x = 0;
        direction.y = 0;
        direction.z = 0;

        //ROS_ERROR_STREAM("obstacle_sensor.quadrant_front"<<quadrant_front[0]<< " " <<quadrant_front[1]<<" "<<quadrant_front[2]<< " "<<quadrant_front[3]);
        //ROS_ERROR_STREAM("quadrant_front"<<quadrant_front[0]<< " " <<quadrant_front[1]<<" "<<quadrant_front[2]<< " "<<quadrant_front[3]);
        //ROS_ERROR_STREAM("quadrant_back"<<quadrant_back[0]<< " " << quadrant_back[1]<< " " <<quadrant_back[2]<<" "<< quadrant_back[3]);
        // Set the x direction the drone should fly in
        if (quadrant_front[0] > 0 || quadrant_front[3] > 0){
            direction.x--;
            direction.y--;
        }
        if (quadrant_front[1] > 0 || quadrant_front[2] > 0){
            direction.x++;
            direction.y--;
        }

        if (quadrant_back[0] > 0 || quadrant_back[3] > 0){
            direction.x--;
            direction.y++;
        }
        if (quadrant_back[1] > 0 || quadrant_back[2] > 0){
            direction.x++;
            direction.y++;
        }

        /*
        // Set the z velocity the drone should fly in
        if (quadrant_front[0] > 0 || quadrant_front[1] > 0)
        velocity.z++;
        if (quadrant_front[2] > 0 || quadrant_front[3] > 0)
        velocity.z--;

        if (quadrant_back[0] > 0 || quadrant_back[1] > 0)
        velocity.z++;
        if (quadrant_back[2] > 0 || quadrant_back[3] > 0)
        velocity.z--;
        */

        double distance_to_pt_front = distance(closest_pt_front.x, closest_pt_front.y, closest_pt_front.z);
        double distance_to_pt_back = distance( closest_pt_back.x, closest_pt_back.y, closest_pt_back.z);
        velocity.x = direction.x*(double)panic_distance/std::max(distance_to_pt_front, distance_to_pt_back);
        velocity.y = direction.y*(double)panic_distance/std::max(distance_to_pt_front, distance_to_pt_back);
        velocity.z = direction.z*(double)panic_distance/std::max(distance_to_pt_front, distance_to_pt_back);
        
        if (std::isnan(velocity.x)) {
            velocity.x = direction.x*(double)panic_distance/.1;

        }
        if (std::isnan(velocity.y)) {
            velocity.y = direction.y*(double)panic_distance/.1;
        }
        if (std::isnan(velocity.z)) {
            velocity.z = direction.z*(double)panic_distance/.1;
        }

        //reset for the next round
        for (int i = 0; i<4; i++) {
            quadrant_back[i] = 0;
            quadrant_front[i] = 0;
        }
        closest_pt_back.x = 0; 
        closest_pt_back.y = 0; 
        closest_pt_back.z = 0; 
        closest_pt_front.x = 0; 
        closest_pt_front.y = 0; 
        closest_pt_front.z = 0; 

        panic_front = false;
        panic_back = false;

        return velocity;
    }
    
    private:
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    pcl::PointXYZ closest_pt_front;
    pcl::PointXYZ closest_pt_back;
    int quadrant_front[4]; // Number of points in each quandrant
    int quadrant_back[4];  // Number of points in each quandrant
    bool panic_front; 
    bool panic_back; 
    float panic_distance;

};

int main(int argc, char** argv)
{
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    ros::init(argc, argv, "panic_pcl");
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
    std::string point_topics[2] = {"/points", "/points_back"};
    ros::Publisher panic_pub = nh.advertise<std_msgs::Bool>("panic_topic", 1);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Vector3>("panic_velocity", 1);
    ros::param::get("/panic_pcl/panic_distance", g_panic_distance);
    ObstacleSensor<PointCloud> obstacle_sensor(nh, point_topics);
    ros::Rate loop_rate(20);
    
    //profiling variables 
    ros::Time main_loop_start_hook_t, main_loop_end_hook_t;
   
    while(ros::ok()){
        main_loop_start_hook_t = ros::Time::now();
        ros::spinOnce(); 
        main_loop_end_hook_t = ros::Time::now();

        panic_pub.publish(obstacle_sensor.get_panic_data());
        if (obstacle_sensor.get_panic_data().data){
            //obstacle_sensor.reset_panic(); 
            geometry_msgs::Vector3 velocity= 
                obstacle_sensor.safe_flight_velocity();
            velocity_pub.publish(velocity);
        }

        g_panic_main_loop += (main_loop_end_hook_t - main_loop_start_hook_t).toSec()*1e9; 
        loop_rate.sleep();
    }
}


