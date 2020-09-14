#! /bin/bash

env_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source ${env_dir}/companion_setup_env_var.sh
set -x 
set -e
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda/lib64" 
export PATH="$PATH:/usr/local/cuda/bin"
source /opt/ros/kinetic/setup.bash

########
# darknet 
########
cd $darknet_base_dir
if [[ ! `git status --porcelain`  ]]; then
    cp $base_dir/build_scripts/darknet.patch $darknet_base_dir	 &&\
    git apply --whitespace=fix darknet.patch
    # TODO turn the next couple of lines to a patch aswell
    sed -i 's/GPU=0/GPU=1/' Makefile &&\
    sed -i 's/OPENCV=0/OPENCV=1/' Makefile &&\
    sed -i 's/\(LDFLAGS+= -L\/usr\/local\/cuda\/lib64\)/\1 -L\/usr\/local\/cuda\/lib64\/stubs /' Makefile 
fi

cd $darknet_base_dir
make -j3
cd $darknet_base_dir && wget -nc https://pjreddie.com/media/files/yolov2.weights

########
# mavbench

########
# mavbench-clone
if [[ ! -d $base_dir/catkin_ws/src ]]; then
mkdir -p $base_dir/catkin_ws/src
cd $base_dir/catkin_ws/
catkin_make
fi

cd $mavbench_apps_base_dir/deps/glog_catkin 
if [[ ! `git status --porcelain`  ]]; then
       git apply $base_dir/build_scripts/glog_catkin.patch 
fi

cd $base_dir/catkin_ws/src 
if [[ ! -d "MAV_apps" ]];then
ln -sf $mavbench_apps_base_dir MAV_apps 
fi

# --- the following need more testing but the purpose is to compress the build
# --- command
# mavbench-build
#first_set="catkin_simple;"\
#"eigen_catkin;glog_catkin;eigen_checks;mav_msgs;profile_manager;mav_comm;nlopt;mavbench_msgs;"\
#"mav_visualization;planning_msgs;mav_trajectory_generation;mav_trajectory_generation_ros;"
#second_set="gflags_catkin;minkindr;minkindr_conversions;volumetric_map_base;volumetric_msgs;"\
#"octomap_world;multiagent_collision_check;"\
#"octomap_server;octomap_mapping;future_collision;publish_imu;follow_trajectory;depth_image_proc;"\
#"control_drone;package_delivery;airsim_img_publisher;kdtree;nbvplanner;"
#mapping_and_sar;follow_the_leader;"

#here on
cd $base_dir/catkin_ws/ &&\
    source /opt/ros/kinetic/setup.bash &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="catkin_simple" -j10 &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="eigen_catkin" -j10 &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="glog_catkin" -j10 &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="eigen_checks" -j10 &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="mav_msgs" -j10 &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="mav_comm" -j10 &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="nlopt" -j10 &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="mav_visualization" -j10 && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="planning_msgs" -j10 && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="mav_trajectory_generation" -j10 && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="mav_trajectory_generation_ros" -j10 && \
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="gflags_catkin" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="minkindr" -j10 && \
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="minkindr_conversions" -j10 && \
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="mavbench_msgs" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="run_time" -j10 && \
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="volumetric_map_base" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="volumetric_msgs" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="profile_manager" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="data_collection" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="octomap_world" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="multiagent_collision_check" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="mavbench_msgs" -j10 && \
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="kdtree" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="compare_octomaps" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="control_drone" -j3 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="volumetric_msgs" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="octomap_server" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="future_collision" -j10
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="publish_imu" -j10 && \
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="follow_trajectory" -j10 && \
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="octomap_mapping" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="depth_image_proc" -j10 && \
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="package_delivery" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="airsim_img_publisher" -j10 
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="nbvplanner" -j10 &&\
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="mapping_and_sar" -j3 && \
#    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="follow_the_leader" -j3 &&\

# --- for eclipse
#    source /opt/ros/kinetic/setup.bash &&\
    catkin_make  --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="gflags_catkin" -j10 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="minkindr" -j10 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="minkindr_conversions" -j10 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="mavbench_msgs" -j10 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="data_collection" -j10 &&\
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="volumetric_map_base" -j10 &&\
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="volumetric_msgs" -j10 &&\
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="profile_manager" -j10 &&
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug  -DCATKIN_WHITELIST_PACKAGES="octomap_world" -j10 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug  -DCATKIN_WHITELIST_PACKAGES="multiagent_collision_check" -j10 &&\
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug  -DCATKIN_WHITELIST_PACKAGES="mavbench_msgs" -j10 &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="kdtree" -j10 &&\
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug  -DCATKIN_WHITELIST_PACKAGES="compare_octomaps" -j10 &&\
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="control_drone" -j3 &&\
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="octomap_server" -j3 &&\
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="future_collision" -j3 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="publish_imu" -j3 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="follow_trajectory" -j3 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="octomap_mapping" -j3 &&\
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="depth_image_proc" -j3 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="depth_image_proc_with_spin" -j3 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="airsim_img_publisher" -j3 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="filter_queue" -j3 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="package_delivery" -j1 && \
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES="run_time" -j10
#
#
#    source /opt/ros/kinetic/setup.bash &&\
#    catkin build  catkin_simple --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  eigen_catkin --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  glog_catkin --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  eigen_checks --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  mav_msgs --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  mav_comm --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  nlopt --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    
#    catkin build  mav_visualization --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  planning_msgs --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  mav_trajectory_generation --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    
#    catkin build  mav_trajectory_generation_ros --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  gflags_catkin --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  minkindr --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  minkindr_conversions --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  mavbench_msgs --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  profile_manager --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  volumetric_map_base --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  volumetric_msgs --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  octomap_world --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  multiagent_collision_check --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  mavbench_msgs --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  octomap_server --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  future_collision --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  publish_imu --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  follow_trajectory --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  octomap_mapping --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  depth_image_proc --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  package_delivery --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  airsim_img_publisher --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  kdtree --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  nbvplanner --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  mapping_and_sar --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  folllow_the_leader --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug &&\
#    catkin build  control_drone --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
#    
