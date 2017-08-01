#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "template_library.hpp"
#include <sstream>
#include "rpc/RpcLibClient.hpp"
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
#include "mavbench/get_trajectory.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>

using namespace std;
bool should_panic = false;
bool future_col = false;

void sigIntHandler(int sig)
{
    ros::shutdown();
    exit(0);
}

double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.y)*(t.x-m.y) + (t.y-m.x)*(t.y-m.x) + (t.z+m.z)*(t.z+m.z));
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
			auto pos = drone.gps();
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " pos: " << pos.x << ", " << pos.y << ", " << pos.z << endl;
        } else if (cmd != "c") {
			cout << "Unknown command" << endl;
		}
	}
}


// *** F:DN call back function for the panic_topic subscriber
void panic_call_back(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
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

    ROS_INFO("Spinning around...");
    drone.set_yaw(90);
    drone.set_yaw(180);
    drone.set_yaw(-90);
    drone.set_yaw(0);
    drone.set_yaw(yaw);

    should_panic = true;
    ROS_INFO("Done panicking!");
}

void action_upon_future_col(Drone& drone) {
    float yaw = drone.get_yaw();

    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    ROS_INFO("Spinning around...");
    drone.set_yaw(yaw+30 <= 180 ? yaw + 30 : 360 - yaw - 30);
    drone.set_yaw(yaw);
    drone.set_yaw(yaw-30 <= 180 ? yaw - 30 : 360 - yaw + 30);
    drone.set_yaw(yaw);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "airsim_planner_demo", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandler);
	
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    double input_x, input_y, input_z; //goal asked by the user
    geometry_msgs::Point start, goal, original_start; //msg send out to the 
	ros::ServiceClient client = n.serviceClient<mavbench::get_trajectory>("get_trajectory_srv");
	mavbench::get_trajectory srv;
	string ip_addr;
    ros::param::get("/airsim_planner_demo/ip_addr",ip_addr);
    uint16_t port = 41451;
    Drone drone(ip_addr.c_str(), port);
     
    // *** F:DN panic subscriber 
    ros::NodeHandle panic_nh;
    ros::Subscriber panic_sub = 
    panic_nh.subscribe<std_msgs::Bool>("panic_topic", 1000, panic_call_back);
    ros::NodeHandle future_col_nh;
    ros::Subscriber future_col_sub = future_col_nh.subscribe<std_msgs::Bool>("future_col_topic", 1000, future_col_callback);

    
    //----------------------------------------------------------------- 
	// *** F:DN knobs
	//----------------------------------------------------------------- 
    double issue_cmd__time_step;  //how often sending a cmd to the drone
    ros::param::get("/airsim_planner_demo/issue_cmd__time_step", issue_cmd__time_step);
    const int step__total_number = 1;
    int points_to_replan_after;
    ros::param::get("/airsim_planner_demo/points_to_replan_after", points_to_replan_after);
    
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    ros::Rate loop_rate(100);
    while (ros::ok())
	{
        // *** F:DN arm, disarm, move around before switching to autonomous mode 
        control_drone(drone);
	    
        // *** F:DN set drone start position	
        auto drone_pos = drone.gps();
		start.x = drone_pos.y; start.y = drone_pos.x; start.z = -drone_pos.z;
		std::cout << "Current position is " << drone_pos.x << " " << drone_pos.y << " " << drone_pos.z << std::endl;
	    
        // *** F:DN set drone goal 	
        std::cout << "Enter goal in following format: x y z" << std::endl;
        std::cin >> input_x >> input_y >> input_z;
		goal.x = input_y; goal.y = input_x; goal.z = -1*input_z;
        original_start = start;
		srv.request.start = start;
		srv.request.goal = goal;

        bool returned_to_start = false;
ugly_loop: //TODO: get rid of this monstrosity. Move this stuff into a function.
        while (dist(drone.gps(), goal) > 5.0) {
            ROS_INFO("Distance to target: %f", dist(drone.gps(), goal));

            auto drone_pos = drone.gps();
            start.x = drone_pos.y; start.y = drone_pos.x; start.z = -drone_pos.z;
            srv.request.start = start;

            // *** F:DN ask for the service
            if (client.call(srv))
            {
                ROS_INFO("Received trajectory.");
            }
            else
            {
                ROS_ERROR("Failed to call service.");
                break;
            }

            if (srv.response.unknown != -1) {
                auto unknown_pos = srv.response.multiDOFtrajectory.points[srv.response.unknown].transforms[0].translation;
                ROS_WARN("Enters unknown space at %f, %f, %f\n", unknown_pos.x, unknown_pos.y, unknown_pos.z); 
            }

            // *** F:DN iterate through cmd propopsed and issue them
            int max_points = points_to_replan_after < srv.response.multiDOFtrajectory.points.size()-1 ? points_to_replan_after : srv.response.multiDOFtrajectory.points.size()-1;

            should_panic = future_col = false;
            int last_point = -1;
            for (int i = 0; !should_panic && i < max_points; ++i) {
                auto p = srv.response.multiDOFtrajectory.points[i];
                auto p_next = srv.response.multiDOFtrajectory.points[i+1];

                double p_x = p.transforms[0].translation.y;
                double p_y = p.transforms[0].translation.x;
                double p_z = -p.transforms[0].translation.z;

                double p_x_next = p_next.transforms[0].translation.y;
                double p_y_next = p_next.transforms[0].translation.x;
                double p_z_next = -p_next.transforms[0].translation.z;

                double v_x = p.velocities[0].linear.y;
                double v_y = p.velocities[0].linear.x;
                double v_z = -p.velocities[0].linear.z;

                double v_x_next = p_next.velocities[0].linear.y;
                double v_y_next = p_next.velocities[0].linear.x;
                double v_z_next = -p_next.velocities[0].linear.z;

                double segment_dedicated_time = (p_next.time_from_start - p.time_from_start).toSec();
                auto segment_start_time = std::chrono::system_clock::now();
                
                double segment__t_step_size = segment_dedicated_time/step__total_number;
                
                for (int step_ctr = 0; step_ctr < step__total_number ; step_ctr +=1) {
                    const double k = 0;

                    double p_x_inter = p_x + (p_x_next - p_x) * (double)step_ctr / step__total_number;
                    double p_y_inter = p_y + (p_y_next - p_y) * (double)step_ctr / step__total_number;
                    double p_z_inter = p_z + (p_z_next - p_z) * (double)step_ctr / step__total_number;

                    double v_x_inter = v_x + (v_x_next - v_x) * (double)step_ctr / step__total_number;
                    double v_y_inter = v_y + (v_y_next - v_y) * (double)step_ctr / step__total_number;
                    double v_z_inter = v_z + (v_z_next - v_z) * (double)step_ctr / step__total_number;

                    auto drone_pos = drone.gps();

                    drone.fly_velocity(v_x_inter + k*(p_x_inter-drone_pos.x),
                            v_y_inter + k*(p_y_inter-drone_pos.y),
                            v_z_inter + 0.2*(p_z_inter-drone_pos.z));

                    ros::spinOnce(); // Check whether we should panic
                    
                    if (should_panic) {
                        ROS_ERROR("you should panic\n");
                        action_upon_panic(drone);
                        break;
                    }

                    if (future_col && last_point == -1) {
                        last_point = i;
                    }

                    std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>((step_ctr+1)*segment__t_step_size));
                }

                if (future_col && last_point != -1 && i > last_point + 8) {
                    ROS_WARN("Obstacle appeared on trajectory");
                    action_upon_future_col(drone);
                    break;
                }
            }
            drone.fly_velocity(0, 0, 0);
        }

        // Now return to start
        if (!returned_to_start) {
            ROS_INFO("Returning to start");
            float yaw = drone.get_yaw();
            drone.set_yaw(yaw + 180);

            start = goal;
            goal = original_start;
            srv.request.start = start;
            srv.request.goal = goal;

            returned_to_start = true;
            goto ugly_loop;
        }

        ROS_INFO("Done!");

		loop_rate.sleep();
	}

	return 0;
}

