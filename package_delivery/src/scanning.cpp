#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "template_library.hpp"
#include <sstream>
//#include "rpc/RpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
//#include "controllers/DroneControllerBase.hpp"
#include "control_drone.h"
#include "common/Common.hpp"
#include <fstream>
#include "Drone.h"
#include "package_delivery/get_trajectory.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>
#include "common.h"
#include "follow_trajectory.h"
using namespace std;
bool should_panic = false;
bool future_col = false;
string ip_addr__global;
string localization_method;


/*
void sigIntHandler(int sig)
{
    ros::shutdown();
    exit(0);
}
*/
double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.y)*(t.x-m.y) + (t.y-m.x)*(t.y-m.x) + (t.z+m.z)*(t.z+m.z));
}


/*
void control_drone(Drone& drone)
{
	cout << "Initialize drone:\n";
	cout << "\ta: arm\n";
	cout << "\td: disarm\n";
	cout << "\tt h: takeoff to h m\n";
	cout << "\tl: land\n";
	cout << "\tf x y z d: fly at (x,y,z) m/s for d s\n";
	cout << "\ty x: set yaw to x\n";
	cout << "\tp: print pitch, roll, yaw, height\n";
	cout << "\tc: complete drone setup and continue\n";
	cout << "\tCtrl-c: quit\n";

	std::string cmd("");

	while(cmd != "c") {
		cin >> cmd;

	    if (cmd == "a") {
	        drone.arm();
		} else if (cmd == "d") {
			drone.disarm();
		} else if (cmd == "t") {
			double height;
			cin >> height;
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
			drone.takeoff(height);
		} else if (cmd == "l") {
			drone.land();
		} else if (cmd == "f") {
			double x,y,z,d;
			cin >> x >> y >> z >> d;
			drone.fly_velocity(x, y, z, d);
		} else if (cmd == "y") {
			double x;
			cin >> x;
			drone.set_yaw(x);
		} else if (cmd == "p") {
			auto pos = drone.pose().position;
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " pos: " << pos.x << ", " << pos.y << ", " << pos.z << endl;
        } else if (cmd != "c") {
			cout << "Unknown command" << endl;
		}
	}
}
*/

// *** F:DN call back function for the panic_topic subscriber
void panic_call_back(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
}

void future_col_callback(const std_msgs::Bool::ConstPtr& msg) {
    future_col = msg->data;
}






void package_delivery_initialize_params() {
    ros::param::get("/scanning/ip_addr",ip_addr__global);
    ros::param::get("/scanning/localization_method",localization_method);
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "scanning", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle panic_nh;
    signal(SIGINT, sigIntHandler);
	printf("ok");
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    string app_name;
    package_delivery_initialize_params();
    double input_x, input_y, input_z; //goal asked by the user
    int n_pts_per_dir; 
    geometry_msgs::Point start, goal, original_start; //msg send out to the 
	package_delivery::get_trajectory get_trajectory_srv;
	
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    
    //bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin
    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        n.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv");
    ros::Subscriber panic_sub =  
		panic_nh.subscribe<std_msgs::Bool>("panic_topic", 1000, panic_call_back);
    ros::NodeHandle future_col_nh;
    //ros::Subscriber future_col_sub = 
    //		future_col_nh.subscribe<std_msgs::Bool>("future_col_topic", 1000, future_col_callback);
    future_col = false; //this is hardcode because scanning does not provide this feature
    
    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
        //const int step__total_number = 1;
    int package_delivery_loop_rate = 100;
    float goal_s_error_margin = 5.0; //ok distance to be away from the goal.
                                                      //this is b/c it's very hard 
                                                      //given the issues associated with
                                                      //flight controler to land exactly
                                                      //on the goal

    
    
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    ros::Rate loop_rate(package_delivery_loop_rate);
    while(ros::ok())
	{
        
	// *** F:DN arm, disarm, move around before switching to autonomous mode 
        control_drone(drone);
	    
        // *** F:DN set drone start position	
        auto drone_pos = drone.pose().position;
		start.x = drone_pos.x; start.y = drone_pos.y; start.z = drone_pos.z;
		std::cout << "Current position is " << drone_pos.x << " " << drone_pos.y << " " << drone_pos.z << std::endl;
	    
        // *** F:DN set drone goal 	
        //std::cout<<"Enter the name of the app that you'd like to try out"<<endl; 
        //std:cin>>app_name; 
        std::cout << "Enter width ,length and number of lanes"<<std::endl
            << "associated with the area you like to sweep "<<endl;
//        std::cout << "program control path selction based on app is IP"<<endl;

        
         
        std::cin >> input_x >> input_y>>n_pts_per_dir;
        
        
        original_start = start;
        get_trajectory_srv.request.start = start;
	    get_trajectory_srv.request.width = (int)input_y;	
	    get_trajectory_srv.request.length = (int)input_x;	
	    get_trajectory_srv.request.n_pts_per_dir = (int)n_pts_per_dir;	
        
        
        //goal.x = 10;
        //goal.y = 10;
        //goal.z = 10;


        //bool returned_to_start = false;
        //spin_around(drone); 
        //ROS_INFO("Distance to target: %f", dist(drone.gps(), goal));
        //auto drone_pos = drone.gps();
        //start.x = drone_pos.y; start.y = drone_pos.x; start.z = -drone_pos.z;
        //get_trajectory_srv.request.start = start;

        // *** F:DN ask for the trajectory
        if (get_trajectory_client.call(get_trajectory_srv))
        {
            ROS_INFO("Received trajectory.");
        }
        else
        {
            ROS_ERROR("Failed to call service.");
            break;
        }


        // *** F:DN iterate through cmd propopsed and issue them
        should_panic = future_col = false;
        int last_point = -1;
        
        //ROS_INFO("size of the trajactory %d",  get_trajectory_srv.response.multiDOFtrajectory.points.size());
        follow_trajecotry(get_trajectory_srv, drone);
        drone.fly_velocity(0, 0, 0);

        ROS_INFO("scanned the entire space and returned successfully");
        //loop_rate.sleep();
    }

    return 0;
}

