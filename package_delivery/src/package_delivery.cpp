#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
//#include "template_library.hpp"
#include <sstream>
//#include "api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
//#include "controllers/DroneControllerBase.hpp"
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
#include "timer.h"

using namespace std;
#include "follow_trajectory.h"
bool should_panic = false;
bool future_col = false;
string ip_addr__global;
string localization_method;


void sigIntHandler(int sig)
{
    ros::shutdown();
    exit(0);
}

double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.x)*(t.x-m.x) + (t.y-m.y)*(t.y-m.y) + (t.z+m.z)*(t.z-m.z));
}



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
	cout << "\tCtrl-c/q: quit\n";

	std::string cmd("");

	while(cmd != "c") {
		cin >> cmd;

            if (cmd == "q") {
              LOG_TIME(package_delivery);
              cout << "bye~" << endl;
              raise(SIGINT);
              return;
            } else if (cmd == "s") {
              sleep(5);
            }

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
			auto pos = drone.gps();
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " pos: " << pos.x << ", " << pos.y << ", " << pos.z << endl;
        } else if (cmd != "c") {
			cout << "Unknown command" << endl;
            ros::shutdown();
            exit(0);
		}
	}
}


// *** F:DN call back function for the panic_topic subscriber
void panic_call_back(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
    should_panic = false;
}

void future_col_callback(const std_msgs::Bool::ConstPtr& msg) {
    future_col = msg->data;
}

void action_upon_panic(Drone& drone) {
    float yaw = drone.get_yaw();

    while (should_panic) {
        drone.fly_velocity(-std::cos(yaw*M_PI/180), -std::sin(yaw*M_PI/180), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        ros::spinOnce();
        ROS_INFO("Panicking..");
    }
    ROS_INFO("Panicking one last time...");
    drone.fly_velocity(-std::cos(yaw*M_PI/180), -std::sin(yaw*M_PI/180), 0, 0.75);
    std::this_thread::sleep_for(std::chrono::milliseconds(850));

    spin_around(drone);
    should_panic = true;
    ROS_INFO("Done panicking!");
}

/*
void action_upon_future_col(Drone& drone) {
    // scan_around(drone, 90);
    scan_around(drone, 20);
}
*/

void package_delivery_initialize_params() {
    ros::param::get("/package_delivery/ip_addr",ip_addr__global);
    ros::param::get("/package_delivery/localization_method",localization_method);
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "package_delivery", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle panic_nh;
    signal(SIGINT, sigIntHandler);
	
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    
    package_delivery_initialize_params();
    double input_x, input_y, input_z; //goal asked by the user
    geometry_msgs::Point start, goal, original_start; //msg send out to the 
	package_delivery::get_trajectory get_trajectory_srv;
	
    uint16_t port = 41451;
    Drone drone(ip_addr__global.c_str(), port);
    int reaction_delay_counter_init_value = 1; 
    int reaction_delay_counter =  reaction_delay_counter_init_value;
    bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin
    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        n.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv");
    ros::Subscriber panic_sub =  
		panic_nh.subscribe<std_msgs::Bool>("panic_topic", 1000, panic_call_back);
    ros::NodeHandle future_col_nh;
    ros::Subscriber future_col_sub = 
		future_col_nh.subscribe<std_msgs::Bool>("future_col_topic", 1000, future_col_callback);

    auto pos_fun = [&]() {
        return drone.position();
    };


    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
        //const int step__total_number = 1;
    int package_delivery_loop_rate = 100;
    float goal_s_error_margin = 2.0; //ok distance to be away from the goal.
                                                      //this is b/c it's very hard 
                                                      //given the issues associated with
                                                      //flight controler to land exactly
                                                      //on the goal

    
    
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    ros::Rate loop_rate(package_delivery_loop_rate);
    LOG_TIME(package_delivery);
    while (ros::ok())
	{
        // *** F:DN arm, disarm, move around before switching to autonomous mode 
        control_drone(drone);

        // Take-off
        /*do {
            float z = pos_fun().z;
            float k = 0.5;
            drone.fly_velocity(0,0, k*(-4 - z));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (-pos_fun().z > 4.5 || -pos_fun().z < 3.5); */
	    
        // *** F:DN set drone start position	
        auto drone_pos = pos_fun();
		start.x = drone_pos.x; start.y = drone_pos.y; start.z = drone_pos.z;
		std::cout << "Current position is " << drone_pos.x << " " << drone_pos.y << " " << drone_pos.z << std::endl;
	    
        // *** F:DN set drone goal 	
        std::cout << "Enter goal in following format: x y z" << std::endl;
        std::cin >> input_x >> input_y >> input_z;
		goal.x = input_x; goal.y = input_y; goal.z = input_z;
        original_start = start;
		get_trajectory_srv.request.start = start;
		get_trajectory_srv.request.goal = goal;

        bool returned_to_start = false;
        spin_around(drone); 

        while (!delivering_mission_complete) {//go toward the destination and come back
            // *** F:DN request client call from the trajectory server and 
            //          follow the path
            while (dist(pos_fun(), goal) > goal_s_error_margin) {
                ROS_INFO("Distance to target: %f", dist(pos_fun(), goal));
                auto drone_pos = pos_fun();
                start.x = drone_pos.x; start.y = drone_pos.y; start.z = drone_pos.z;
                get_trajectory_srv.request.start = start;

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

                /* 
                   if (get_trajectory_srv.response.unknown != -1) {
                   auto unknown_pos = get_trajectory_srv.response.multiDOFtrajectory.points[get_trajectory_srv.response.unknown].transforms[0].translation;
                   ROS_WARN("Enters unknown space at %f, %f, %f\n", unknown_pos.x, unknown_pos.y, unknown_pos.z); 
                   }
                   */

                // Look towards target
                double dx = goal.x - drone_pos.x;
                double dy = goal.y - drone_pos.y;
                //ROS_INFO("Turning to %f degrees", std::atan(dy / dx) * 180 / M_PI);
                drone.set_yaw(std::atan(dx / dy) * 180 / M_PI);

                // *** F:DN iterate through cmd propopsed and issue them
                should_panic = future_col = false;
                int last_point = -1;
                
                follow_trajecotry(get_trajectory_srv, drone);
                /*
                for (int i = 0; !should_panic && i <get_trajectory_srv.response.multiDOFtrajectory.points.size()-1; ++i) {
                    auto p = get_trajectory_srv.response.multiDOFtrajectory.points[i];
                    auto p_next = get_trajectory_srv.response.multiDOFtrajectory.points[i+1];

                    double p_z = -p.transforms[0].translation.z;
                    
                    double v_x = p.velocities[0].linear.y;
                    double v_y = p.velocities[0].linear.x;
                    double v_z = -p.velocities[0].linear.z;

                    double segment_dedicated_time = (p_next.time_from_start - p.time_from_start).toSec();
                    auto segment_start_time = std::chrono::system_clock::now();

                    drone.fly_velocity(v_x, 
                            v_y,
                            v_z + 0.2*(p_z-pos_fun().z));

                    ros::spinOnce(); // Check whether we should panic

                    if (should_panic) { //if deteceted a panic
                        ROS_ERROR("you should panic\n");
                        action_upon_panic(drone);
                        break;
                    }

                    if (future_col){ //if detect a future colision, replan(but wait a bit)
                        if(reaction_delay_counter <= 0) {
                            ROS_WARN("Obstacle appeared on trajectory");
                            action_upon_future_col(drone);
                            reaction_delay_counter = reaction_delay_counter_init_value; 
                            break;
                        }
                        reaction_delay_counter--;
                    }

                    std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>((1)*segment_dedicated_time));
                }
                */
                drone.fly_velocity(0, 0, 0);
            }

            if (returned_to_start) { //if returned to start, we are done
                delivering_mission_complete = true;
            } else { //else, adjust the goal to return back
                ROS_INFO("Returning to start");
                float yaw = drone.get_yaw();
                drone.set_yaw(yaw + 180 <= 180 ? yaw + 180 : -(360 - yaw - 180));

                start = goal;
                goal = original_start;
                ROS_INFO_STREAM("her eis my start "<<start);
                ROS_INFO_STREAM("her eis my start "<<goal);
                
                get_trajectory_srv.request.start = start;
                get_trajectory_srv.request.goal = goal;

                returned_to_start = true;
            }
        }

        ROS_INFO("Delivered the package and returned!");
        delivering_mission_complete = false;

        // Get collision info
        // int collisions = drone.collisions();
        // ROS_INFO("Collision count: %d", collisions);

        loop_rate.sleep();
    }

    return 0;
}

