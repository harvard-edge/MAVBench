# Building MAVBench and Using it

NOTE1: Please read untill the end before setting up your system. 

NOTE2: Please setup the companion computer before the host.

# Companion Computer  (Responsible for running the compute intensive workloads)

## System Requirements
**Hardware**:  
+ Jetson TX2  

**Software**:  
+ Ubuntu: 16.04  
+ JetPack: 3.2 (We have only tested our setup with 3.2 but we suspect, it'll work with higher versions as well)  

## Building It 
1. git clone  --recursive https://github.com/MAVBench/MAVBench.git MAVBench_base
2. cd MAVBench_base
2. source build-scripts/companion_setup_env_var.sh
3. sudo ./build-scripts/companion_root_setup.bash 
4. ./build-scripts/companion_user_setup.bash


### Build Notes :
- If the user wants to manually build some of our ROS packages using catkin, they need to make sure to source setup_var_env.sh first  


# Host Computer (Responsible for running the drone/environment simualtors + autopilot subsystem)

## System Requirements
**Hardware**:  
+ A system with powerfull CPU + GPU (Our tested setup uses an Intel Core i7 CPU and a high-end NVIDIA GTX 1080 Ti GPU).

**Software**:  
+ Windows 10, 64 bit   (at thist point, we only support windows for the host)
+ Python 2 (also make sure pip is installed)
+ Visual Studio (optional: only if you want to build from scratch) (tested with visual studio 15.8, 2017 community eddition)  
+ Unreal (optional: only if you want to build from scratch) ( tested with 4.18) 


## How to Build 
git clone  --recursive https://github.com/MAVBench/MAVBench.git mavbench_base_dir

**For the lazy yet happy**: We have provided a set of games (environments drone can fly within) that can be simply exectued by the user. To do so:
1. cd MAVBench_base/build-scripts
2. host_setup_env_var.cmd 
3.host_root_setup.cmd

**For the reckless with no life** (most likely you won't fall within this group): In case the user wants to build from scratch (this can be helpful if the user wants to try out new environment maps), follow the insurction provided by Microsoft (https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md) only replacing the **Build AirSim** section with the following instructinos:
1. You will need Visual Studio 2017 (make sure to install VC++ and Windows SDK 8.x).
2. Start x64 Native Tools Command Prompt for VS 2017. 
3.Create a folder for the repo (here on refered to as MAVBench_base) and
5. cd MAVBench_base/build-scripts 
6. host_setup_env_var.cmd 
7. host_root_setup_from_src.cmd
7. This will create ready to use plugin in MAVBench_base/src/AirSim/Unreal/Plugins folder that can be dropped into any Unreal project 
- follow along with the AirSim instuctions provided by Microsoft https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). 
Notes to internal developers: 
      1. If you decided to make your own executable and upload to google drive, use windows to zip and windows to unzip (you can use tar in windows now). I had issues with using tar for unziping.The generated executable was erroring out.
      2. a folder with the name of game/WindowsNoEditor need to be zipped to the google cloud

### fixing AirSims depth map issue
(*This step may have a problem) Fix a Depth image bug by following this issue: https://github.com/Microsoft/AirSim/issues/491. 
1. Go to BP_PIPCamera (within the unreal editor. This is located under Blueprints (Note: if you can’t find this in the content broweser, click on window->Find in blueprints and serach for BP_RIPCamera)
1. Click on DepthPlannerCaptureComponent (on the left hand side under Components tab). Then in the Details window, click on “post process Materials” and change the material to “DepthMapMaterial”

![alt text](https://github.com/MAVBench/MAVBench/blob/master/docs/images/BP_PIP_depth-map-modification.PNG)


