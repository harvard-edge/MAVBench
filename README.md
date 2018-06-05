# MAV-Bench

## To download the source code:

Run the following commands (replacing `$catkin_ws` with the location of your catkin workspace and `hngenc` with your own username):
```shell
cd $catkin_ws/src
git clone https://hngenc@bitbucket.org/zaddan/mav-bench.git
```

## To install pre-reqs

Install the following packages, replacing `kinetic` with your own ROS distribution:

Run the following commands:
```shell
cd $catkin_ws/src/mav-bench
./prereqs.sh
```

### Build PCL with C++11 support

One of our pre-requisites, the PCL library, is not normally built with C++11 support, but some of our packages require it to be. Therefore, it must be built manually from source.

First, clone the repository, and checkout PCL 1.7.2:
```shell
git clone https://github.com/PointCloudLibrary/pcl.git
git checkout pcl-1.7.2rc2.1
```

Next, add the following line to PCL's CMakeLists.txt file: `SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")`

Finally, build and install the project by following the official instructions: http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php

It is possible that the binaries will not be installed in the directories where catkin expects them to be. In those cases, a quick fix is to simply copy the libpcl\*.so binaries from the directory where they were installed to the directory where catkin is looking for them. You will be able to tell from catkin's error messages where they are supposed to be. _(TODO: find the correct place to install the binaries)._


## To install SLAM packages

### first install glew
sudo apt-get install libglew-dev
### then install pangolan(follow the instruction on their github page)
### ORB-SLAM2
Run the following commands (replacing $catkin_ws with the location of your catkin workspace and hngenc with your own username):
```shell
cd $catkin_ws/src/mav-bench
git clone https://hngenc@bitbucket.org/hngenc/orb_slam2.git
cd orb_slam2 
./build.sh (if this step fails, it's possibly that the processor ran out of ram. The soltion is to chance the build.sh to force the build process to use less cores (e.g. make -j1))
./build_ros.sh
```

To run ORB-SLAM2, run the following command:
```shell
cd $catkin_ws/src/mav-bench/orb_slam2
./scripts/run_rgbd.sh cameras/airsim-rgbd.yaml
```

Right now, we only have support for ORB-SLAM2's RGBD mode.

## Package-delivery

### To build the package_delivery package:

Finally, navigate to your catkin workspace and build:
```shell
cd $catkin_ws
catkin_make
```

If you are on the TX1/2, you may have to run the following command to build glog:
```shell
wget -O $catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src/config.guess 'http://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.guess;hb=HEAD'
wget -O $catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src/config.sub 'http://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.sub;hb=HEAD'
```

### To run the package_delivery package:
change the host_ip (located in the package_delivery.launch file to the ip of the host (where airsim is running))
```shell
roslaunch package_delivery package_delivery.launch
```

