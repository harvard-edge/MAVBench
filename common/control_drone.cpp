#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "template_library.hpp"
#include <sstream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
#include <fstream>
#include "Drone.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <time.h>
#include "common.h"
#include <cstring>
#include <string>

using namespace std;

void control_drone(Drone& drone)
{
	cout << "Initialize drone:\n";
	cout << "\ta: arm\n";
	cout << "\td: disarm\n";
	cout << "\tt h: takeoff to h m\n";
	cout << "\tl: land\n";
	cout << "\tf x y z d: fly at (x,y,z) m/s for d s\n";
	cout << "\tfz x y z d: fly at (x,y) m/s for d s, holding height z m\n";
	cout << "\ty x: set yaw to x\n";
	cout << "\tyz x z: set yaw to x degrees, holding height z m\n";
	cout << "\tp: print pitch, roll, yaw, height\n";
	cout << "\tc: complete drone setup and continue\n";
	cout << "\ts: sleep for 5 seconds\n";
	cout << "\tr: rotate slowly\n";
    cout << "\tCtrl-c/q: quit\n";

	std::string cmd("");

	while(cmd != "c") {
		cin >> cmd;
        if (cmd == "q") {
          //LOG_TIME(package_delivery);
          cout << "bye~" << endl;
          ros::shutdown();
          exit(0);
          return;
        }

	    
        if (cmd == "a") {
            drone.arm();
        } else if (cmd == "s") {
			double t;
			cin >> t;
            sleep(t);
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
			drone.fly_velocity(x, y, z, YAW_UNCHANGED, d);
        } else if (cmd == "fz") {
			double x,y,z,d;
			cin >> x >> y >> z >> d;
			drone.fly_velocity_at_z(x, y, z, YAW_UNCHANGED, d);
		} else if (cmd == "y") {
			double x;
			cin >> x;
			drone.set_yaw(x);
		} else if (cmd == "yz") {
			double y, z;
			cin >> y >> z;
			drone.set_yaw_at_z(y, z);
		} else if (cmd == "p") {
			auto pos = drone.pose().position;
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " pos: " << pos.x << ", " << pos.y << ", " << pos.z << endl;
        } else if (cmd == "r") {
            spin_around(drone); 
        } else if (cmd != "c") {
			cout << "Unknown command" << endl;
            // ros::shutdown();
            // exit(0);
		}
	}

    // Print flight summary at start of execution
    std::string fname;
    if (ros::param::get("/stats_file_addr", fname)) {
        //update_stats_file(fname, "\nFlightSummaryStart: ");
        //output_flight_summary(drone, fname);
    } else {
        ROS_ERROR("No stats_file found");
    }
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
	cout << "\tr: rotate slowlyd\n";
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
        } else if (cmd == "r") {
            spin_around(drone); 
        }else if (cmd != "c") {
			cout << "Unknown command" << endl;
		}
	}
}
*/

