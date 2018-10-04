#! /bin/bash

env_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source ${env_dir}/setup_env_var.sh

set -x 
set -e

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda/lib64" 
export PATH="$PATH:/usr/local/cuda/bin"
source /opt/ros/kinetic/setup.bash

# darknet
cd $mavbench_base_dir
if [[ ! -d "darknet" ]]; then
    cd $mavbench_base_dir && git clone https://github.com/pjreddie/darknet.git &&\
    cd darknet && git checkout d528cbdb7bf58c094026377aa80c26971d0ae1b0
fi

cd $mavbench_base_dir/darknet
if [[ ! `git status --porcelain`  ]]; then
    cp $mavbench_base_dir/darknet.patch $mavbench_base_dir/darknet/	 &&\
    git apply --whitespace=fix darknet.patch
    # TODO turn the next couple of lines to a patch aswell
    sed -i 's/GPU=0/GPU=1/' Makefile &&\
    sed -i 's/OPENCV=0/OPENCV=1/' Makefile &&\
    sed -i 's/\(LDFLAGS+= -L\/usr\/local\/cuda\/lib64\)/\1 -L\/usr\/local\/cuda\/lib64\/stubs /' Makefile 
fi

cd $mavbench_base_dir/darknet
make -j3
cd $mavbench_base_dir/darknet && wget -nc https://pjreddie.com/media/files/yolov2.weights



# mavbench
mkdir -p $mavbench_base_dir/catkin_ws/src
cd $mavbench_base_dir/catkin_ws/src
#
listethzOfRepos="eigen_catkin eigen_checks glog_catkin mav_comm nlopt \
                 mav_trajectory_generation gflags_catkin \
                 minkindr minkindr_ros"

cd $mavbench_base_dir/catkin_ws/src
for repo in $listethzOfRepos
do
    if [[ ! -d $repo ]];then
        git clone https://github.com/ethz-asl/"$repo".git
    fi
done


if [[ ! -d "catkin_simple" ]];then
    git clone https://github.com/catkin/catkin_simple.git
fi 

cd $mavbench_base_dir/catkin_ws/src
if [[ ! -d "perception_pcl" ]];then
    git clone "https://github.com/ros-perception/perception_pcl.git"
fi

if [[ ! -d "publishAirsimImgs" ]];then
    git clone https://github.com/marcelinomalmeidan/publishAirsimImgs.git
fi

cd $mavbench_base_dir/catkin_ws/src
if [[ ! -d "mav-bench" ]];then
    git clone --recursive  https://github.com/hngenc/mav-bench.git  &&  \
    cd mav-bench && \
    git checkout -b refactor origin/refactor
fi

cd $mavbench_base_dir/catkin_ws/src/glog_catkin 
if [[ ! `git status --porcelain`  ]]; then
       git checkout de911f71cb832dcc0668bca56727b4b7b1e42126 && \
       git apply $mavbench_base_dir/catkin_ws/src/mav-bench/misc/glog_catkin.patch 
fi

cd $mavbench_base_dir/catkin_ws/src/mav_trajectory_generation 
git checkout ee318fc2478a04c85a95e96dedb4a7be6731720c

cd $mavbench_base_dir/catkin_ws/src/mav_comm
git checkout 521b2b21ffb6c86e724a9b6144b0171a371c9ee4 


cd $mavbench_base_dir/catkin_ws/ &&\
    source /opt/ros/kinetic/setup.bash &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="catkin_simple" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="eigen_catkin" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="glog_catkin" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="eigen_checks" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="mav_msgs" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="mav_comm" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="nlopt" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="mav_visualization" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="planning_msgs" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="mav_trajectory_generation" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="mav_trajectory_generation_ros" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="gflags_catkin" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="minkindr" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="minkindr_conversions" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="mavbench_msgs" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="volumetric_map_base" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="volumetric_msgs" &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="profile_manager" -j3 &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="octomap_world" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="multiagent_collision_check" -j3 &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="mavbench_msgs" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="octomap_server" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="future_collision" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="publish_imu" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="follow_trajectory" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="octomap_mapping" -j3 &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="depth_image_proc" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="package_delivery" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="airsim_img_publisher" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="kdtree" -j3 &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="nbvplanner" -j3 &&\
    catkin_make -DCATKIN_WHITELIST_PACKAGES="mapping_and_sar" -j3 && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="follow_the_leader" -j3
   

#--- probably need to delete the following
##RUN	cp $mavbench_base_dir/ros-kinetic-opencv3_3.3.1-5xenial_arm64.deb /usr/src/deb/ &&\
##    dpkg -i $mavbench_base_dir/ros-kinetic-opencv3_3.3.1-5xenial_arm64.deb 

