# Companion Computer  

## Running It
- Set the host_ip in setup_env_var.sh to the host ip.
- source MAVBench_base/buil-scripts/companion_setup_env_var.sh 
- source MAVBench_base/catkin_ws/devel/setup.bash

- use rolaunch to interact with the individual applications that is roslaunch $pkg_name $application.launch:
  
  example: roslaunch package_delivery scanning.launch
- Note: all our applications require a set of (what we call) pre-mission steps to prime them. We have provided a pre_mission-cmd.sh for each application, so to run an application with pre-mission commands use: 
  
./MAVbench_base/src/mav-bench-apps/$application_name/pre_mission_cmds.sh | roslaunch $pkg_name $application.launch 


### Running Notes:
- If the user has manually built our ROS packages, they need to set all the variabiles in teh companion_setup_env_var.sh accordingly.

- For internal developer Notes: 'c' needs to be pressed always after everything is loaded, so e.g. if it takes a long time for the object detection to get loaded, c needs to be pressed according.- 

# Host Computer

## How to run 
**For the lazy yet happy** (method 1):
1. cd MAVBench_base/games/WindowsNoEditor
2. execute the .exe

**For the reckless with no life** (method 2):
ollow the insurction provided by Microsoft (https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). Follow the **How to Use Airsim** Section.  


# Runnin Hello World
- HelloWorld: You can run the hello world with: ./MAVbench_base/src/mav-bench-apps/control_drone/pre_mission_cmds.sh | roslaunch control_drone control_drone.launch. What you should see if running this program is the drone navigatin a square around a warehouse  


