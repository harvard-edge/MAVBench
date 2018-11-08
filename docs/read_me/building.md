This document describes the steps necessary for building MAVBench toolset.    
Note: Please read until the end before setting up your system.   
Note: Please setup the companion computer before the host.  

[comment]: <p align="center"> 
# Companion Computer 

This computer is responsible for running the compute intensive workloads.

## System Requirements
**Hardware**:  
+ Jetson TX2  

**Software**:  
+ Ubuntu: 16.04  
+ JetPack (Nvidia SDK): 3.2 (We have only tested our setup with 3.2 but we suspect, it'll work with higher versions as well)  

## Building It 
```bash
git clone  --recursive https://github.com/MAVBench/MAVBench.git MAVBench_base;     
cd MAVBench_base;   
source build-scripts/companion_setup_env_var.sh;    
sudo ./build-scripts/companion_root_setup.bash;    
./build-scripts/companion_user_setup.bash;  
```    

### Build Notes :
- If the user wants to manually build some of our ROS (robotic operating system) packages using catkin, they need to make sure to source setup_var_env.sh first.  

# Host Computer 
This computer is responsible for running the drone/environment simulators + autopilot subsystem).

## System Requirements
**Hardware**:  
+ A system with powerful CPU + GPU (Our tested setup uses an Intel Core i7 CPU and a high-end NVIDIA GTX 1080 Ti GPU).

**Software**:  
+ Windows 10, 64 bit   (at this moment, we only support windows for the host)
+ Python 2 (also make sure pip is installed)
+ Visual Studio (optional: only if you want to build from scratch) (tested with visual studio 15.8, 2017 community edition)  
+ Unreal (optional: only if you want to build from scratch) ( tested with 4.18) 


## Building It
1.  
> git clone  --recursive https://github.com/MAVBench/MAVBench.git mavbench_base_dir;   
   
   **For the lazy yet happy**: We have provided a set of games (environments drone can fly within) that can be simply executed by the user. To do so:  
    2.
> cd MAVBench_base/build-scripts;  
    host_setup_env_var.cmd;  
    host_root_setup.cmd              
   
   **For the reckless with no life** (most likely you won't fall within this group): 
   By building from scratch the user can try out their own environment maps. Inorder to do so, folow the instruction provided by Microsoft (https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). Note that you will need Visual Studio 2017 (make sure to install VC++ and Windows SDK 8.x). Replace the **Build AirSim** section with the following instructions:    
   2.  Start x64 Native Tools Command Prompt for VS 2017.       
   >  mkdir MAVBench_base;  
   > cd MAVBench_base/build-scripts;  
   > host_setup_env_var.cmd;  
   > host_root_setup_from_src.cmd;  
   
   This will create ready to use plugin in MAVBench_base/src/AirSim/Unreal/Plugins folder that can be dropped into any Unreal project. Follow along with the AirSim instructions provided by Microsoft   https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md).       

### Build Notes :
for internal developers: 
If you decided to make your own executable and upload to google drive, use windows to zip and windows to unzip (you can use tar in windows now). I had issues with using tar for unziping. The generated executable was erroring out.A folder with the name of game/WindowsNoEditor need to be zipped to the google cloud.

### Fixing AirSim's Depth Map Issue
Fix a Depth image bug by following this issue: https://github.com/Microsoft/AirSim/issues/491. 
1. Go to BP_PIPCamera (within the unreal editor, this is located under Blueprints (Note: if you can’t find this in the content browser, click on "window->Find" in blueprints and search for BP_RIPCamera)
1. Click on DepthPlannerCaptureComponent (on the left hand side under "Components" tab). Then in the "Details" window, click on “post process Materials” and change the material to “DepthMapMaterial”

![alt text](https://github.com/MAVBench/MAVBench/blob/master/docs/images/BP_PIP_depth_map_modification.PNG)



## Building Games (mainly for internal developers):
Steps to create, upload and deploy games: (for internal developers):
1. make a game in unreal.
2. package it.
3.  zip it (I usually right "click->send to->compressed" (zipped) folder. I believe 7zip should work too, but further investigation is required).
4. uploaded it to the google drive.
5. get a shareable link and paste the id (what's after "id" in the shared link before the next "/") to host_setup_env_var.sh game_fileid variable.


