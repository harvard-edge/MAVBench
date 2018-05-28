cd ..

# ROS packages
sudo apt-get install ros-kinetic-octomap*
sudo apt-get install ros-kinetic-depth-image-proc
sudo apt-get install ros-kinetic-stereo-msgs
sudo apt-get install ros-kinetic-tf-conversions
sudo apt-get install ros-kinetic-rviz-visual-tools
sudo apt-get install ros-kinetic-vision-opencv
sudo apt-get install ros-kinetic-ompl

# Pre-reqs to side-step PCL C++11 issue
git clone https://github.com/ros-perception/perception_pcl.git

# Prereqs for mav_trajectory_generation, our trajectory-smoothening package
#git clone https://github.com/catkin/catkin_simple.git
#git clone https://github.com/ethz-asl/eigen_catkin.git
#git clone https://github.com/ethz-asl/eigen_checks.git
#git clone https://github.com/ethz-asl/glog_catkin.git
#git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/nlopt.git
git clone https://github.com/ethz-asl/mav_trajectory_generation.git

# Prereqs for SAR
git clone --recursive https://github.com/ethz-asl/nbvplanner.git
#git clone https://github.com/ethz-asl/volumetric_mapping.git
#git clone https://github.com/ethz-asl/gflags_catkin.git
#git clone https://github.com/ethz-asl/minkindr.git
#git clone https://github.com/ethz-asl/minkindr_ros.git
rm -r nbvplanner/nbvplanner
# Package to publish Airsim imgs and camera info
git clone https://github.com/marcelinomalmeidan/publishAirsimImgs.git



