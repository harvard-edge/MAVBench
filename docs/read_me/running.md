This document describes the steps necessary for running the MAVBench toolset. 
# Companion Computer  

## Running It
1. Set the host_ip in setup_env_var.sh to the host IP.   
2. Source the relevant files.
```bash
source MAVBench_base/buil-scripts/companion_setup_env_var.sh;    
source MAVBench_base/catkin_ws/devel/setup.bash;    
```
3.Use roslaunch to interact with the individual applications:   
```bash
roslaunch $pkg_name $application.launch #example: roslaunch package_delivery scanning.launch;
``` 
 
4.At this point, you can interact with the applications
4(alternative). you can use our pre-defined missions (encapsulating a set of initial interactions) to prime the drone for a specific goal. These pre-defined missions are provided for each application in a file called pre_mission_cmds.sh: 
```bash
#example: ./MAVbench_base/src/mav-bench-apps/package_delivery/pre_mission_cmds.sh | roslaunch $package_delivery $scanning.launch 
./MAVbench_base/src/mav-bench-apps/$application_name/pre_mission_cmds.sh; | roslaunch $pkg_name $application.launch;  
```


### Running Notes:
- If the user has manually built our ROS packages, they need to set all the variables in the companion_setup_env_var.sh accordingly.

- Note (for internal developers): "c" needs to be pressed always after all the processes(nodes) are loaded, so e.g. if it takes a long time for the object detection to get loaded, pressing "c" needs to be postponed accordingly. 

# Host Computer

## Running it 
**For the lazy yet happy** (method 1):
1. Switch to the appropriate directory. 
```bash
cd MAVBench_base/games/WindowsNoEditor
```
2. Execute the .exe

**For the reckless with no life** (method 2):
Follow the instruction provided by Microsoft (https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). Follow the **How to Use Airsim** Section.  


# Running Hello World
You can run the hello world with: 
```basH
./MAVbench_base/src/mav-bench-apps/control_drone/pre_mission_cmds.sh | roslaunch control_drone control_drone.launch. 
```
while running, you shoud see the drone navigating a square around a warehouse, returning to initial coordinates and land.  


