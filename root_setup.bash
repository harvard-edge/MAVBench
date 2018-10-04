#!/bin/bash
source env.bash
#export mavbench_base_dir=$PWD

# spit out commands to stdout
set -x 
# stop at any error
set -e 

# for some reason this is not necessary in the docker (but otherwise pcl
# is gonna issue an error
cd /usr/lib/aarch64-linux-gnu/
sudo ln -sf tegra/libGL.so libGL.so

# should remove so if the previous runs have failed, it woulnd't cause an 
# issue
rm -f /etc/apt/sources.list.d/ros-latest.list # if we don't remove, we can't be 
                                              # sure which stage last tim

#--- update/upgrade and install relevant packages
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
echo "deb-src http://packages.ros.org/ros/ubuntu xenial main" >> /etc/apt/sources.list.d/ros-latest.list

apt-get update
##--- install ros-kinetic-desktop-full
apt-get install -y ros-kinetic-desktop-full ros-kinetic-rviz-visual-tools

# intall ROS OpenCV
if [[ ! -d "$mavbench_base_dir/opencv" ]];then
	cd $mavbench_base_dir && \
		git clone -b 3.1.0-with-cuda8 https://github.com/daveselinger/opencv opencv
fi

# the following needs to change so that it's not specific to a version of opencv
# (if we want to stick to a version we need to provide the source code ourselves
cd $mavbench_base_dir 
if [[ ! -e "ros-kinetic-opencv3_3.3.1-5xenial_arm64.deb" ]];then
    cd $mavbench_base_dir 
    rm -f ros-kinetic-opencv3-3.3.1  
    apt-get source ros-kinetic-opencv3
	cp $mavbench_base_dir/opencv/modules/cudalegacy/src/graphcuts.cpp $mavbench_base_dir/ros-kinetic-opencv3-3.3.1/modules/cudalegacy/src/graphcuts.cpp
    # Dependencies
    cd $mavbench_base_dir/ros-kinetic-opencv3-3.3.1 && \
	   apt-get build-dep -y ros-kinetic-opencv3
    # Now build (we ignore missing dependencies, because we have them on our system anyways)
    sed -i 's/\(\bdh_shlibdeps.*\)$/\1 --dpkg-shlibdeps-params=--ignore-missing-info/' debian/rules || exit 1
	dpkg-buildpackage -b -uc 
fi

 
if [[ ! -e ros_install_done.txt ]]; then
    rm -f /usr/src/deb_mavbench 
    mkdir /usr/src/deb_mavbench
	cp $mavbench_base_dir/ros-kinetic-opencv3_3.3.1-5xenial_arm64.deb /usr/src/deb_mavbench/
    cd /usr/src/deb_mavbench/
    chmod a+wr /usr/src/deb_mavbench && \
	apt-ftparchive packages . | gzip -c9 > Packages.gz && \
	apt-ftparchive sources . | gzip -c9 > Sources.gz && \
	chmod a+wr /etc/apt/sources.list.d/ros-latest.list && \
	echo "deb file:/usr/src/deb_mavbench ./" >> /etc/apt/sources.list.d/ros-latest.list && \
	sed -i -e "1,2s/^/#/g" /etc/apt/sources.list.d/ros-latest.list && \
	apt-get update && \
	apt-get remove -y ros-kinetic-opencv3 && \
	apt-get install -y ros-kinetic-opencv3 --allow-unauthenticated && \
	sed -i -e "s/#//g" /etc/apt/sources.list.d/ros-latest.list && \
	apt-get update && \
	apt-get install -y ros-kinetic-desktop-full ros-kinetic-rviz-visual-tools sudo apt-get install -y ros-kinetic-octomap*
    cp /opt/ros/kinetic/lib/aarch64-linux-gnu/pkgconfig/opencv-3.3.1-dev.pc /opt/ros/kinetic/lib/aarch64-linux-gnu/pkgconfig/opencv.pc 
    echo "done" > ros_install_done.txt
fi

cd $mavbench_base_dir
#--- point cloud library
if [[ ! -d "pcl" ]]; then
    cd $mavbench_base_dir/ && git clone https://github.com/PointCloudLibrary/pcl.git &&\
    cd pcl && git checkout pcl-1.7.2rc2.1
fi

cd $mavbench_base_dir/pcl
if [[ ! `git status --porcelain`  ]]; then
    cp $mavbench_base_dir/lzf_image_io.cpp $mavbench_base_dir/pcl/io/src/ 
fi

cd $mavbench_base_dir/pcl && mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-std=c++11" ..
cd $mavbench_base_dir/pcl/build && make -j 4
cd $mavbench_base_dir/pcl/build && make -j 4 install

# TODO: this is gonna cause linker for libs that use pcl to run everytime
#       somehow, we need to figure out how to avoid doing it everytime
cd $mavbench_base_dir/ && chmod +x relocate_pcl.sh && ./relocate_pcl.sh

# airsim
cd $mavbench_base_dir
if [[ ! -d "AirSim" ]];then
    cd $mavbench_base_dir/ && git clone https://github.com/hngenc/AirSim.git
    git fetch origin && \
    git checkout -b future_darwing_dev origin/future_darwing_dev 
fi    

cd $mavbench_base_dir/AirSim &&\
    ./setup.sh && \
    ./build.sh

