/**
* octomap_server_nodelet: A nodelet version of A. Hornung's octomap server
* @author Marcus Liebhardt
* License: BSD
*/

/*
 * Copyright (c) 2012, Marcus Liebhardt, Yujin Robot
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <octomap_server/OctomapServer.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
//#include <signal.h>
#include "profile_manager/start_profiling_srv.h"
#include "profile_manager/profiling_data_srv.h"

namespace octomap_server
{


static OctomapServer  *glbl_ptr;
void log_data_before_shutting_down(){
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    profiling_data_srv_inst.request.key = "img_to_octomap_commun_t";
    profiling_data_srv_inst.request.value = ((double)glbl_ptr->pt_cld_octomap_commun_overhead_acc/1e9)/glbl_ptr->octomap_ctr;
     
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager using octomap");
            ros::shutdown();
        }
    }
     
    //ROS_ERROR_STREAM("oooooooook"<<glbl_ptr->octomap_ctr);
    //ROS_ERROR_STREAM("---------------------- BLAAAAAAAAAAAAAAAH33"<<((double)glbl_ptr->pt_cld_octomap_commun_overhead_acc/1e9)/glbl_ptr->octomap_ctr); 
    
    /* 
    std::string ns = ros::this_node::getName();
    profiling_data_srv_inst.request.key = "octomap_integration";
    profiling_data_srv_inst.request.value = (((double)octomap_integration_acc)/1e9)/octomap_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager using octomap");
            ros::shutdown();
        }
    }
    
    ROS_ERROR_STREAM("---------------------- BLAAAAAAAAAAAAAAAH4"); 
   */ 
    
    /*
    profiling_data_srv_inst.request.key = "octomap_main_loop";
    profiling_data_srv_inst.request.value = (((double)g_accumulate_loop_time)/1e9)/g_main_loop_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
        }
    }
    */
}
/*
void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        ros::shutdown();
    }
    //exit(0);
}
*/
class OctomapServerNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    
  glbl_ptr =   server_.get();
  NODELET_DEBUG("Initializing octomap server nodelet ...");
    ros::NodeHandle& private_nh = this->getPrivateNodeHandle();
    server_.reset(new OctomapServer(private_nh));

    //signal(SIGINT, sigIntHandlerPrivate);

    std::string mapFilename("");
    if (private_nh.getParam("map_file", mapFilename)) {
      if (!server_->openFile(mapFilename)){
        NODELET_WARN("Could not open file %s", mapFilename.c_str());
      }
    }
  }
 

private:
  boost::shared_ptr<OctomapServer> server_;
};

} // namespace

PLUGINLIB_EXPORT_CLASS(octomap_server::OctomapServerNodelet, nodelet::Nodelet)
