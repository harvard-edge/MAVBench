#! /bin/bash

env_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source ${env_dir}/setup_env_var.sh

set -x 
set -e

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda/lib64" 
export PATH="$PATH:/usr/local/cuda/bin"
source /opt/ros/kinetic/setup.bash

# darknet
#cd $mavbench_base_dir
#if [[ ! -d "darknet" ]]; then
#    cd $mavbench_base_dir && git clone https://github.com/pjreddie/darknet.git &&\
#    cd darknet && git checkout d528cbdb7bf58c094026377aa80c26971d0ae1b0
#fi
#
cd $darknet_base_dir
if [[ ! `git status --porcelain`  ]]; then
    cp $base_dir/build-scripts/darknet.patch $darknet_base_dir	 &&\
    git apply --whitespace=fix darknet.patch
    # TODO turn the next couple of lines to a patch aswell
    sed -i 's/GPU=0/GPU=1/' Makefile &&\
    sed -i 's/OPENCV=0/OPENCV=1/' Makefile &&\
    sed -i 's/\(LDFLAGS+= -L\/usr\/local\/cuda\/lib64\)/\1 -L\/usr\/local\/cuda\/lib64\/stubs /' Makefile 
fi

cd $darknet_base_dir
make -j3
cd $darknet_base_dir && wget -nc https://pjreddie.com/media/files/yolov2.weights


# mavbench
if [[ ! -d $base_dir/catkin_ws/src ]]; then
mkdir -p $base_dir/catkin_ws/src
cd $base_dir/catkin_ws/
catkin_make
fi
#cd $mavbench_base_dir
#if [[ ! -d "mav-bench" ]];then
#    git clone --recursive  -b refactor https://github.com/hngenc/mav-bench.git 
#fi
#
cd $mavbench_apps_base_dir/deps/glog_catkin 
if [[ ! `git status --porcelain`  ]]; then
       git apply $base_dir/build-scripts/glog_catkin.patch 
fi

cd $base_dir/catkin_ws/src 
if [[ ! -d "mav-bench" ]];then
ln -sf $mavbench_apps_base_dir mav-bench 
fi
#
#
cd $base_dir/catkin_ws/ &&\
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
#   

#--- probably need to delete the following
##RUN	cp $mavbench_base_dir/ros-kinetic-opencv3_3.3.1-5xenial_arm64.deb /usr/src/deb/ &&\
##    dpkg -i $mavbench_base_dir/ros-kinetic-opencv3_3.3.1-5xenial_arm64.deb 
  

#--- probably need to delete the following
##RUN	cp $mavbench_base_dir/ros-kinetic-opencv3_3.3.1-5xenial_arm64.deb /usr/src/deb/ &&\
##    dpkg -i $mavbench_base_dir/ros-kinetic-opencv3_3.3.1-5xenial_arm64.deb 

