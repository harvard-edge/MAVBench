This document describes the steps necessary for running the MAVBench toolset. 
# Companion Computer  

## Running It
1. Set the host_ip in companion_setup_env_var.sh to the host IP.   
2. Source the relevant files.
```bash
cd MAVBench_base;
source build_scripts/companion_setup_env_var.sh;    
source catkin_ws/devel/setup.bash;    
```
3.Use roslaunch to interact with the individual applications:   
```bash
roslaunch $pkg_name $application.launch #example: roslaunch package_delivery scanning.launch;
``` 
Note that for the application to communicate with the game, the game needs to be running on the host computer (so follow the instruction on the host at this point to get the game running).

4.At this point, you can interact with the applications
4(alternative). you can use our pre-defined missions (encapsulating a set of initial interactions) to prime the drone for a specific goal. These pre-defined missions are provided for each application in a file called pre_mission_cmds.sh: 
```bash
#example: ./MAVbench_base/src/MAV_apps/package_delivery/pre_mission_cmds.sh | roslaunch $package_delivery $scanning.launch 
./MAVbench_base/src/MAV_apps/$application_name/pre_mission_cmds.sh; | roslaunch $pkg_name $application.launch;  
```


### Running Notes:
- If the user has manually built our ROS packages, they need to set all the variables in the companion_setup_env_var.sh accordingly.

- Note (for internal developers): "c" needs to be pressed always after all the processes(nodes) are loaded, so e.g. if it takes a long time for the object detection to get loaded, pressing "c" needs to be postponed accordingly. 

# Host Computer

## Running it 
**For the lazy yet happy** (method 1):

Open the Developer Command Prompt for VS 2017
1. Switch to the appropriate directory. 
```bash
cd MAVBench_base_dir\test_benches\configs
```
2.Modify the json config file as you like to invoke the desired game for the application of interest.
Note: you only need to set the map_name in the json file (we have provided 5 maps, 1 for each application. The names are ${application_name}_simple, so for example for package delivery set the map_name to package_delivery_simple).

3. start the game
```bash
cd MAVBench_base_dir\test_benches;
python scripts\run_games.py --config configs\${your_config_file} #example:  python scripts\run_games.py --config configs\run_game_config.json
```
**For the reckless with no life** (method 2):
Follow the instruction provided by Microsoft (https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). Follow the **How to Use Airsim** Section.  


# Running Hello World
You can run the hello world with: 
```basH
./MAVbench_base/src/MAV_apps/control_drone/pre_mission_cmds.sh | roslaunch control_drone control_drone.launch. 
```
while running, you shoud see the drone navigating a square around a warehouse, returning to initial coordinates and land.  


