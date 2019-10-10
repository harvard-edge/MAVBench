#include <Drone.h>
#include <future_collision/future_collision.h>
#include <octomap_server/OctomapServer.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <rosconsole/macros_generated.h>
#include <signal.h>
#include <XmlRpcValue.h>
#include <cstdint>
#include <cstdlib>
#include <string>

#include "motion_planner.h"

FutureCollisionChecker * fcc_ptr = nullptr;
MotionPlanner * mp_ptr = nullptr;
void sigIntHandlerPrivate(int signo) {
    if (signo == SIGINT) {
        fcc_ptr->log_data_before_shutting_down();
        mp_ptr->log_data_before_shutting_down();
        ros::shutdown();
    }
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_map_node");
    ros::NodeHandle nh("~");
    std::string mapFilename(""), mapFilenameParam("");
    signal(SIGINT, sigIntHandlerPrivate);

    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    
    // Create a Drone object
    std::string ip_addr, localization_method;
    ros::param::get("/ip_addr", ip_addr);
    uint16_t port = 41451;
    if(!ros::param::get("/localization_method", localization_method)) {
        ROS_FATAL("Could not start occupancy map node. Localization parameter missing!");
        exit(-1);
    }
    Drone drone(ip_addr.c_str(), port, localization_method);
    
    // Create an octomap server
    octomap_server::OctomapServer server;
    octomap::OcTree * octree = server.tree_ptr();


    if (nh.getParam("/occupancy_map_node/map_file", mapFilenameParam)) {
        if (mapFilename != "") {
            ROS_WARN("map_file is specified by the argument '%s' and rosparam '%s'. now loads '%s'", mapFilename.c_str(), mapFilenameParam.c_str(), mapFilename.c_str());
        } else {
            mapFilename = mapFilenameParam;
        }
    }
    if (mapFilename != "") {
        if (!server.openFile(mapFilename)){
            ROS_ERROR("Could not open file %s", mapFilename.c_str());
            exit(1);
        }
    }

    // Create FutureCollisionChecker
    FutureCollisionChecker fcc (octree, &drone);
    fcc.setOctomapServer(&server); // This is only used for profiling purposes.
    fcc_ptr = &fcc;

    // Create MotionPlanner
    //MotionPlanner mp (octree, &drone);
    //mp_ptr = &mp;

    while (ros::ok()) {
        // ROS_INFO("Start octo");
        // ros::Time start_octo = ros::Time::now();
        ros::spinOnce();
        // ros::Time end_octo = ros::Time::now();
        // ROS_INFO("End octo");
        // std::cout << "octomap takes " << (end_octo - start_octo).toSec() << "\n";

        // ros::Time start_fcc = ros::Time::now();
        fcc.spinOnce();

     //   ros::Time now_secs =ros::Time::now();
//        ROS_ERROR_STREAM("now time is "<<now_secs.toSec());
 //       ros::Duration(5).sleep();
  //      ros::Time after_now_secs =ros::Time::now();
   //     ROS_ERROR_STREAM("after now time is "<<now_secs.toSec());
    //    ROS_ERROR_STREAM("diff is "<<(after_now_secs - now_secs).toSec());

        // ros::Time end_fcc = ros::Time::now();
        // std::cout << "fcc takes " << (end_fcc - start_fcc).toSec() << "\n";

        // ROS_INFO("Start mp");
        // ros::Time start_mp = ros::Time::now();
        //mp.spinOnce();
        // ROS_INFO("End mp");
        // ros::Time end_mp = ros::Time::now();
        // std::cout << "mp takes " << (end_mp - start_mp).toSec() << "\n";
    }
}

