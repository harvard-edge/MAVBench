#include "ros/ros.h"

// Standard headers
#include <chrono>
#include <string>
#include <cmath>
#include <signal.h>

// ROS message headers
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"

// Octomap specific headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

// MAVBench headers
#include "Drone.h"
#include "timer.h"
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <package_delivery/BoolPlusHeader.h>
#include <package_delivery/multiDOF_array.h>

// Typedefs
typedef package_delivery::multiDOF_array traj_msg_t;
typedef std::chrono::system_clock sys_clock;
typedef std::chrono::time_point<sys_clock> sys_clock_time_point;
static const sys_clock_time_point never = sys_clock_time_point::min();

// Profiling variables
ros::Time start_hook_chk_col_t, end_hook_chk_col_t;                                          
long long g_checking_collision_kernel_acc = 0;
ros::Time g_checking_collision_t;
long long g_future_collision_main_loop = 0;
int g_check_collision_ctr = 0;
double g_distance_to_collision_first_realized = 0;
bool CLCT_DATA = false;
bool DEBUG = false;
ros::Time g_pt_cloud_header; //this is used to figure out the octomap msg that 
						  //collision was detected in

ros::Duration g_pt_cloud_to_future_collision_t; 

bool g_got_new_traj = false;

// Global variables
octomap::OcTree * octree = nullptr;
traj_msg_t traj;
std::string ip_addr__global;
std::string localization_method;
double drone_height__global;
double drone_radius__global;



template <class T>
bool collision(octomap::OcTree * octree, const T& n1, const T& n2)
{
	const double pi = 3.14159265359;

    const double height = drone_height__global; 
    const double radius = drone_radius__global; 

	const double angle_step = pi/4;
	const double radius_step = radius/3;
	const double height_step = height/2;

	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;
	double dz = n2.z - n1.z;

	double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

	octomap::point3d direction(dx, dy, dz);
	octomap::point3d end;

	for (double h = -height/2; h <= height/2; h += height_step) {
		for (double r = 0; r <= radius; r += radius_step) {
			for (double a = 0; a <= pi*2; a += angle_step) {
				octomap::point3d start(n1.x + r*std::cos(a), n1.y + r*std::sin(a), n1.z + h);

				if (octree->castRay(start, direction, end, true, distance)) {
					return true;
				}
			}
		}
	}

	return false;
}


template <class T>
double dist_to_collision(Drone& drone, const T& col_pos) {
    auto drone_pos = drone.position();

	double dx = drone_pos.x - col_pos.x;
	double dy = drone_pos.y - col_pos.y;
	double dz = drone_pos.z - col_pos.z;

	return std::sqrt(dx*dx + dy*dy + dz*dz);
}

template <class T>
bool in_safe_zone(const T& start, const T& pos) {
    const double radius = drone_radius__global;
    const double height = drone_height__global;

	double dx = start.x - pos.x;
	double dy = start.y - pos.y;
	double dz = start.z - pos.z;

    return (std::sqrt(dx*dx + dy*dy) < radius && std::abs(dz) < height);
}

void pull_octomap(const octomap_msgs::Octomap& msg)
{
    //ROS_INFO_STREAM("now octo"); 
    RESET_TIMER();
    if (octree != nullptr) {
        delete octree;
    }

	octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(msg);
	octree = dynamic_cast<octomap::OcTree*> (tree);

    if (octree == nullptr) {
        ROS_ERROR("Octree could not be pulled.");
    }
    g_pt_cloud_header = msg.header.stamp; 
	LOG_ELAPSED(future_collision_pull);
}

void new_traj(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
    g_got_new_traj = true;
}

void pull_traj(Drone& drone, const traj_msg_t::ConstPtr& msg)
{
    auto pos = drone.position();
    const auto& traj_front = msg->points.front();
    double x_offset = pos.x - traj_front.x;
    double y_offset = pos.y - traj_front.y;
    double z_offset = pos.z - traj_front.z;

    traj = *msg;
    for (auto& point : traj.points){
        point.x += x_offset;
        point.y += y_offset;
        point.z += z_offset;
    }
}

/* int seconds (sys_clock_time_point t) {
    auto int_s = std::chrono::time_point_cast<std::chrono::seconds>(t);
    return int_s.time_since_epoch().count() % 50;
} */

bool check_for_collisions(Drone& drone, sys_clock_time_point& time_to_warn)
{

    start_hook_chk_col_t = ros::Time::now();

    RESET_TIMER();

    const double min_dist_from_collision = 100.0;
    const std::chrono::milliseconds grace_period(1500);

    if (octree == nullptr || traj.points.size() < 1) {
        //ROS_INFO_STREAM("shouldn't be here"); 
        return false;
    }
    //ROS_INFO_STREAM("now check for colision");
    // auto& start = traj.points[0].transforms[0].translation;

    bool col = false;

    for (int i = 0; i < traj.points.size() - 1; ++i) {
        auto& pos1 = traj.points[i]; // .transforms[0].translation;
        auto& pos2 = traj.points[i+1]; // .transforms[0].translation;

        // We ignore possible at the beginning of the journey
        // if (in_safe_zone(start, pos1))
        //     continue;

        if (collision(octree, pos1, pos2)) {
            col = true;

            // Check whether the drone is very close to the point of collision
            auto now = sys_clock::now();
            if (dist_to_collision(drone, pos1) < min_dist_from_collision){
                time_to_warn = now;
               	if(CLCT_DATA){ 
                    g_distance_to_collision_first_realized = dist_to_collision(drone, pos1);
				}
            }
            // Otherwise, give the drone a grace period to continue along its
            // path. Don't update the time_to_warn if it's already been set to
            // some time in the future
            else if (time_to_warn == never) {
                time_to_warn = now + grace_period;
                // ROS_ERROR("now: %d", seconds(now));
                //  ROS_ERROR("time_to_warn: %d", seconds(time_to_warn));
            }

            break;
        }
    }

    if (!col)
        time_to_warn = never;

    LOG_ELAPSED(future_collision);
    
    end_hook_chk_col_t = ros::Time::now(); 
    g_checking_collision_t = end_hook_chk_col_t;
    g_checking_collision_kernel_acc += ((end_hook_chk_col_t - start_hook_chk_col_t).toSec()*1e9);
    g_check_collision_ctr++;
    return col;
}


void future_collision_initialize_params()
{
    ros::param::get("/motion_planner/drone_radius", drone_radius__global);
    ros::param::get("/motion_planner/drone_height", drone_height__global);

    if(!ros::param::get("/package_delivery/ip_addr",ip_addr__global)){
        ROS_FATAL("Could not start exploration. IP address parameter missing!");
        return;
    }

    if(!ros::param::get("/package_delivery/localization_method",localization_method)){
        ROS_FATAL("Could not start exploration. Localization parameter missing!");
        return; 
    }
	if(!ros::param::get("/CLCT_DATA",CLCT_DATA)){
        ROS_FATAL("Could not start exploration. Localization parameter missing!");
        return; 
    }
    if(!ros::param::get("/DEBUG",DEBUG)){
        ROS_FATAL("Could not start exploration. Localization parameter missing!");
        return; 
    }

}

void log_data_before_shutting_down(){

    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    profiling_data_srv_inst.request.key = "future_collision_kernel";
    profiling_data_srv_inst.request.value = (((double)g_checking_collision_kernel_acc)/1e9)/g_check_collision_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "future_collision_main_loop";
    profiling_data_srv_inst.request.value = (((double)g_future_collision_main_loop)/1e9)/g_check_collision_ctr;
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
    ros::init(argc, argv, "future_collision");
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    
    enum State {checking_for_collision, waiting_for_response};
    future_collision_initialize_params(); 

    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);

    bool collision_coming = false;
    auto time_to_warn = never;

    package_delivery::BoolPlusHeader col_coming_msg;
    std_msgs::Bool col_imminent_msg;

    ros::Subscriber octomap_sub = nh.subscribe("octomap_binary", 1, pull_octomap);
    ros::Subscriber new_traj_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("multidoftraj", 1, new_traj);
    ros::Subscriber traj_sub = nh.subscribe<traj_msg_t>("next_steps", 1, boost::bind(pull_traj, boost::ref(drone), _1));

    ros::Publisher col_coming_pub = nh.advertise<package_delivery::BoolPlusHeader>("col_coming", 1);
    ros::Publisher col_imminent_pub = nh.advertise<std_msgs::Bool>("col_imminent", 1);

	// for profiling
    ros::Publisher octomap_header_pub = nh.advertise<std_msgs::Int32>("octomap_header_col_detected", 1);

    State state, next_state;
    next_state = state = checking_for_collision;
    //profiling variables
    ros::Time main_loop_start_hook_t, main_loop_end_hook_t;
    
    //----------------------------------------------------------------- 
    // *** F:DN BODY
    //----------------------------------------------------------------- 
    ros::Rate loop_rate(60);
    while (ros::ok()) {
        
        main_loop_start_hook_t = ros::Time::now();
        
        ros::spinOnce();
        
        if (state == checking_for_collision) {
            collision_coming = check_for_collisions(drone, time_to_warn);
            if (collision_coming) {
                next_state = waiting_for_response;
                // Publish whether or not a future collision has been detected
                col_coming_msg.header.stamp = g_pt_cloud_header;
                col_coming_msg.data = collision_coming;
                col_coming_pub.publish(col_coming_msg);
                g_got_new_traj = false; 
                // Profiling 
                if(CLCT_DATA){ 
                    g_pt_cloud_to_future_collision_t = start_hook_chk_col_t - g_pt_cloud_header;
                } 
                if(DEBUG) {
                    ROS_INFO_STREAM("pt cloud to start of future collision"<< g_pt_cloud_to_future_collision_t);//<< " " <<start_hook_chk_col_t<<" " << g_pt_cloud_header);
                    //ROS_INFO_STREAM("collision detection time in future_collision"<<g_checking_collision_t);
                }
            }
        }else if (state == waiting_for_response) {
            if (g_got_new_traj){
                next_state = checking_for_collision;
            }
        }
        
        state = next_state;
        //profiling 
		
        // Publish whether or not a collision is close enough that we should
        // respond to it
        /* 
        if (collision_coming) {
            auto now = sys_clock::now();
            if (now >= time_to_warn) {
                col_imminent_msg.data = true;
				col_imminent_pub.publish(col_imminent_msg);
            }
        } else {
            col_imminent_msg.data = false;
            col_imminent_pub.publish(col_imminent_msg);
        }
        */
        
        main_loop_end_hook_t = ros::Time::now();
        g_future_collision_main_loop += (main_loop_end_hook_t - main_loop_start_hook_t).toSec()*1e9; 
        
        loop_rate.sleep();
    }
}

