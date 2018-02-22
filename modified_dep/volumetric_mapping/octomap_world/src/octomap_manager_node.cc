/*
Copyright (c) 2015, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>
#include <ros/package.h>
#include <profile_manager/profiling_data_srv.h>
#include <stdio.h>
#include "octomap_world/octomap_manager.h"
#include <signal.h>


float pc_insertion_rate = 0;
long long g_accumulate_loop_time_ms = 0; //it is in ms
int g_loop_ctr = 0; 



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

    profiling_data_srv_inst.request.key = "octomap_pc_insertation_rate";
    profiling_data_srv_inst.request.value = pc_insertion_rate;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    ros::shutdown();
}


void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        ros::shutdown();
    }
    exit(0);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "octomap_manager");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Time loop_start_t(0,0);
  ros::Time loop_end_t(0,0);
  signal(SIGINT, sigIntHandlerPrivate);

  volumetric_mapping::OctomapManager manager(nh, nh_private);

   bool start_profiling = false;
   while(ros::ok) {
       loop_start_t = ros::Time::now();
       if (!start_profiling) {

       }
       ros::spinOnce();
       
       loop_end_t = ros::Time::now();
       if (loop_end_t.isValid()) {
           if (start_profiling) { 
               g_accumulate_loop_time_ms += ((loop_end_t - loop_start_t).toSec())*1000;
               g_loop_ctr++; 
           }
       }
   }
  return 0;
}
