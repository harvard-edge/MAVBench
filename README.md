# MAV-Bench

## To download the source code:

Run the following commands (replacing `$catkin_ws` with the location of your catkin workspace and `hngenc` with your own username):
	cd $catkin_ws/src
	git clone https://hngenc@bitbucket.org/zaddan/mav-bench.git

## To build the motion-planning package:

Run the following commands:
	cd $catkin_ws/src/MAV-Bench
	./prereqs.sh

Then, install the following packages, replacing `kinetic` with your own ROS distribution:
	sudo apt-get install ros-kinetic-octomap*
	sudo apt-get install ros-kinetic-depth-image-proc

Finally, navigate to your catkin workspace and build:
	cd $catkin_ws
	catkin_make

If you are on the TX1/2, you may have to run the following command to build glog:
	wget -O $catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src/config.guess 'http://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.guess;hb=HEAD'
	wget -O $catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src/config.sub 'http://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.sub;hb=HEAD'


## To run the motion-planning package:
	roslaunch mavbench path_planner.launch

