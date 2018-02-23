#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nbvplanner/nbvp.hpp>
#include <signal.h>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>

long long g_accumulate_loop_time_ms = 0; //it is in ms
int g_loop_ctr = 0; 
bool g_start_profiling = false; 

void log_data_before_shutting_down(){
    profile_manager::profiling_data_srv profiling_data_srv_inst;

    std::string ns = ros::this_node::getName();
    profiling_data_srv_inst.request.key = ns+"_mean_loop_time";
    profiling_data_srv_inst.request.value = ((g_accumulate_loop_time_ms)/1000)/g_loop_ctr;
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

int main(int argc, char** argv)
{
  
 //visualization_msgs::Marker origin_destList;
 ros::init(argc, argv, "nbvPlanner");
 ros::NodeHandle nh;
 ros::NodeHandle nh_private("~");
 signal(SIGINT, sigIntHandlerPrivate);
 bool clct_data = true;
 ros::ServiceClient start_profiling_client = 
     nh.serviceClient<profile_manager::start_profiling_srv>("/start_profiling");

  profile_manager::start_profiling_srv start_profiling_srv_inst;

 nbvInspection::nbvPlanner<Eigen::Matrix<double, 4, 1> > planner(nh, nh_private);
 ros::Time loop_start_t(0,0); 
 ros::Time loop_end_t(0,0); //if zero, it's not valid

 while(ros::ok) {
     loop_start_t = ros::Time::now();

     ros::spinOnce();

     loop_end_t = ros::Time::now(); 

     if (clct_data) { 
         if (ros::service::waitForService("/start_profiling", 10)){ 
             if(!start_profiling_client.call(start_profiling_srv_inst)){
                 ROS_ERROR_STREAM("could not probe data using stats manager");
                 ros::shutdown();
             }
             //ROS_INFO_STREAM("now it is true");
             g_start_profiling = start_profiling_srv_inst.response.start; 
         }
     } 
     if (g_start_profiling){ 
         if (loop_end_t.isValid()) {
             g_accumulate_loop_time_ms += ((loop_end_t - loop_start_t).toSec())*1000;
             g_loop_ctr++; 
         }
     }
 }
 
 return 0;
}
