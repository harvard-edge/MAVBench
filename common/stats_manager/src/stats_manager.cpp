#include "ros/ros.h"

#include <iostream>
#include <signal.h>
#include <string>
#include <sstream>

#include "Drone.h"
#include "common.h"

using namespace std;

// *** F:DN main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "stats_manager");
    ros::NodeHandle n;
 
    uint16_t port = 41451;
    string ip_addr;
    string stats_fname;

    ros::param::get("/ip_addr", ip_addr);
    ros::param::get("/stats_file_addr", stats_fname);

    Drone drone(ip_addr.c_str(), port);

	ros::Rate loop_rate(2);
    while (ros::ok()) {
        loop_rate.sleep();
    }

    update_stats_file(stats_fname, "\nFlightSummaryEnd: ");
    output_flight_summary(drone, stats_fname);

    return 0;
}

