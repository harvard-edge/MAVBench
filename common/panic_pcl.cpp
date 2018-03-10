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
    ObstacleSensor(ros::NodeHandle& nh, const std::string& obstacle_data_topic)
    {
        panic_pub = nh.advertise<std_msgs::Bool>("panic_topic", 1);
        direction_pub = nh.advertise<geometry_msgs::Vector3>("panic_direction", 1);

        sub = nh.subscribe<ObstacleData>(obstacle_data_topic, 1, &ObstacleSensor::callback, this);

        ros::param::get("/panic_pcl/panic_distance", panic_distance);
    }

    void callback(const typename ObstacleData::ConstPtr& msg)
    {
        
        start_hook_t = ros::Time::now();
        std_msgs::Bool panic_msg;
        panic_msg.data = false;

        int quadrant[4] = {0, 0, 0, 0}; // Number of points in each quandrant

        for (const pcl::PointXYZ& pt : msg->points) {
            if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
                if (distance(pt.x, pt.y, pt.z) < panic_distance) {
                    panic_msg.data = true;

                    if (pt.x > 0 && pt.z > 0)
                        quadrant[0]++;
                    else if (pt.x < 0 && pt.z > 0)
                        quadrant[1]++;
                    else if (pt.x < 0 && pt.z < 0)
                        quadrant[2]++;
                    else if (pt.x > 0 && pt.z < 0)
                        quadrant[3]++;
                }
            }
        }

        // A vector representing the direction the drone should fly in to avoid
        // the obstacle. 0 is x, 1 is y, and 2 is z.
        geometry_msgs::Vector3 direction = safe_flight_direction(quadrant);

        end_hook_t = ros::Time::now();
        g_panic_kernel_acc += ((end_hook_t - start_hook_t).toSec()*1e9);
        g_panic_ctr++;

        panic_pub.publish(panic_msg);

        if (panic_msg.data)
            direction_pub.publish(direction);
    
    }

private:
    geometry_msgs::Vector3 safe_flight_direction(const int quadrant[4])
    {
        geometry_msgs::Vector3 direction;

        direction.x = 0;
        direction.y = -1;
        direction.z = 0;

        // Set the x direction the drone should fly in
        if (quadrant[0] > 0 || quadrant[3] > 0)
            direction.x--;
        if (quadrant[1] > 0 || quadrant[2] > 0)
            direction.x++;

        // Set the z direction the drone should fly in
        if (quadrant[0] > 0 || quadrant[1] > 0)
            direction.z++;
        if (quadrant[2] > 0 || quadrant[3] > 0)
            direction.z--;

        return direction;
    }

    ros::Subscriber sub;
    ros::Publisher panic_pub;
    ros::Publisher direction_pub;

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
    
    ObstacleSensor<PointCloud> obstacle_sensor(nh, "/points");
    ros::Rate loop_rate(20);
    
    //profiline variables 
    ros::Time main_loop_start_hook_t, main_loop_end_hook_t;
   
    while(ros::ok()){
        main_loop_start_hook_t = ros::Time::now();
        ros::spinOnce(); 
        main_loop_end_hook_t = ros::Time::now();
        g_panic_main_loop += (main_loop_end_hook_t - main_loop_start_hook_t).toSec()*1e9; 
        loop_rate.sleep();
    }
}


