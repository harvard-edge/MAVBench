# MAV-Bench

## To download the source code:

Run the following commands (replacing `$catkin_ws` with the location of your catkin workspace and `hngenc` with your own username):
```shell
cd $catkin_ws/src
git clone https://hngenc@bitbucket.org/zaddan/mav-bench.git
```

## To install SLAM packages

### ORB-SLAM2
Run the following commands (replacing $catkin_ws with the location of your catkin workspace and hngenc with your own username):
```shell
cd $catkin_ws/src/mav-bench
git clone https://hngenc@bitbucket.org/hngenc/orb_slam2.git
cd orb_slam2
./build.sh
./build_ros.sh
```

To run ORB-SLAM2, run the following command:
```shell
cd $catkin_ws/src/mav-bench/orb_slam2
./scripts/run_rgbd.sh cameras/airsim-rgbd.yaml
```

Right now, we only have support for ORB-SLAM2's RGBD mode.

## To build the package_delivery package:
Run the following commands:
```shell
cd $catkin_ws/src/mav-bench
./prereqs.sh
```

Then, install the following packages, replacing `kinetic` with your own ROS distribution:
```shell
sudo apt-get install ros-kinetic-octomap*
sudo apt-get install ros-kinetic-depth-image-proc
```

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

## To run the package_delivery package:
change the host_ip (located in the package_delivery.launch file to the ip of the host (where airsim is running))
```shell
roslaunch package_delivery package_delivery.launch
```

