#include "ros/ros.h"
#include <octomap_world/octomap_manager.h>
#include <iostream>

int coverage(volumetric_mapping::OctomapManager* manager, int minX, int minY, 
        int minZ, int maxX, int maxY, int maxZ)
{
  
    int covered = 0;
  int uncovered = 0; 
  const double disc = manager->getResolution();
  Eigen::Vector3d vec;
  for (vec[0] = minX;
      vec[0] < maxX; vec[0] +=disc) {
      for (vec[1] = minY;
        vec[1] < maxY; vec[1] += disc) {
        for (vec[2] = minZ;
          vec[2] < maxZ; vec[2] += disc) {
        
        double probability;
        volumetric_mapping::OctomapManager::CellStatus node = manager->getCellProbabilityPoint(
           vec, &probability);
        if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown ) {
            uncovered +=1;
            // Rayshooting to evaluate inspectability of cell
         } else {
             covered +=1;
         }
        
        }
    }
  }
  return ((float)covered/(float)(uncovered + covered))*100;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "compare_octomaps"); 
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string ns = ros::this_node::getName();
  int minX, minY, minZ, maxX, maxY, maxZ;
  //---get params
  if(!ros::param::get(ns+"/minX", minX)){
      ROS_FATAL("Could not compare octomaps. Parameter missing! Looking for %s", 
              (ns + "/minX").c_str());
      return -1; 
  } 

  if(!ros::param::get(ns+"/minY", minY)){
      ROS_FATAL("Could not compare octomaps. Parameter missing! Looking for %s", 
              (ns + "/minY").c_str());
      return -1; 
  } 


  if(!ros::param::get(ns+"/minZ", minZ)){
      ROS_FATAL("Could not compare octomaps. Parameter missing! Looking for %s", 
              (ns + "/minZ").c_str());
      return -1; 
  } 


  if(!ros::param::get(ns+"/maxX", maxX)){
      ROS_FATAL("Could not compare octomaps. Parameter missing! Looking for %s", 
              (ns + "/maxX").c_str());
      return -1; 
  } 

  if(!ros::param::get(ns+"/maxY", maxY)){
      ROS_FATAL("Could not compare octomaps. Parameter missing! Looking for %s", 
              (ns + "/maxY").c_str());
      return -1; 
  } 

  if(!ros::param::get(ns+"/maxZ", maxZ)){
      ROS_FATAL("Could not compare octomaps. Parameter missing! Looking for %s", 
              (ns + "/maxZ").c_str());
      return -1; 
  } 

  ros::Rate loop_rate(10);
  
  volumetric_mapping::OctomapManager *manager = new volumetric_mapping::OctomapManager(nh, nh_private);
  while(ros::ok()){  
      manager->publishAll();
      ROS_INFO_STREAM("here is the coverage"<<coverage(manager, minX, minY, minZ, maxX, maxY, maxZ)<<std::endl);
      ros::spinOnce();
      //ros::shutdown();   
      loop_rate.sleep();
  }
}
