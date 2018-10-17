# Welcome to MAVBench  (Note: We plan to release the code, build scripts and a docker by Micro conference 2018(Oct 20th). As is, the code needs to get cleaned up and the repos might be merged, so please STAY TUNED!!!)
This README explains how to setup and use MAVBench.

FORMATTING TODOs: change all the bulles for instructions to numbers.

TODO: upload all the videos right here


**What is MAVBench?**
MAVBench is a framework targetting design and development of Micro Aerial Vehicles for roboticists, sofware and hardware designers. It consists of a closed-loop simulator and an end-to-end application
benchmark suite. A closed-loop simulation platform is needed to probe and understand the intra-system (application data flow) and inter-system (system and environment) interactions in MAV applications
to pinpoint bottlenecks and identify opportunities for hardware and software co-design and optimization. In addition to the simulator, MAVBench provides a benchmark suite, the first of its kind,
consisting of a variety of MAV applications designed to enable computer architects to perform characterization and develop future aerial computing systems. This work is built on top of a host of open source software.
A big shout out to Microsoft and University of EHTH zurich. 

**Why MAVBench**
Autonomous drones similar to other autonomous machines require a new breed of architectural simulators. Unlike in traditional architectural simulators, information in an autonomous machine flows in a loop. Information flows in from the machine's environment via its sensors, gets processed by the computing subsystem, and flows back out into the environment via actuators and controls. Hence, autonomous machines require a tightly coupled closed-loop feedback system for architectural investigation.

We developed MAVBench---a framework consisting of a hardware-in-the-loop simulator and a set of end-to-end benchmarks. To accurately model the drone's system and its environment, our simulator has three core components (\Fig{fig:end-to-end}). The drone's environments, sensors, and actuators are simulated using a game engine called Unreal augmented with AirSim libraries  (\Fig{fig:end-to-end}, top). By using a physics engine, they provide the ability to simulate the drone's behavior, its environment and the interaction between the two such as accurate collision detection. The flight controller (flight stack and the autopilot hardware) is responsible for the drone's stabilization (\Fig{fig:end-to-end}, bottom right). We use a software-simulated flight controller provided by AirSim. However, AirSim also supports other FCs, such as the Pixhawk. Much of the drone's perception and trajectory planning is done using an onboard computer, which is generally responsible for running any compute-intensive workloads (\Fig{fig:end-to-end}, bottom left). We used an NVIDIA Jetson TX2, although our setup allows for swapping this embedded board with other platforms like a RISC-V based platform. 

![alt text](https://github.com/MAVBench/MAVBench/blob/master/docs/images/end-to-end-simulation.png)


## Youtube Channel
https://www.youtube.com/channel/UC_bNkXcP5BHSRcNJ4R4GTvg

## Demos
[![Watch the video](https://www.youtube.com/watch?v=B_hOO7o0-Bk)

## Building MAVBench and Using it

NOTE1: Please read till the end before setting up your system. 

NOTE2: Please setup the companion computer before the host.

## Companion Computer  (C
### System Requirements
**Hardware**:  
+ Jetson TX2  

**Software**:  
+ Ubuntu: 16.04  
+ JetPack: 3.2 (we have only tested our setup with 3.2 but we suspect, it'll work with higher versions as well)  

### How to Build 
- git clone  --recursive https://github.com/MAVBench/MAVBench.git MAVBench_base
- cd MAVBench_base
- source build-scripts/setup_env_var.sh
- sudo ./build-scripts/companion_root_setup.bash 
- ./build-scripts/companion_user_setup.bash
#### build notes :
- the usr might have to populate the .ssh with public/private key (for both root and usr) //not sure about this though
- TODO: we need to make sure we build all the pkgs with -DCMAKE_BUILD_TYPE=Release
- if the user wants to build pkgs using catkin, he/she needs to make sure to source setup_var_env.sh first
- augment setup_env_var to export host_ip which will be used by all the aps. 

### How to RUN 
- make sure you have set the host_ip in setup_env_var.sh
source MAVBench_base/buil-scripts/setup_env_var.sh 
source MAVBench_base/catkin_ws/devel/setup.bash

Note: make sure that mavbench_base_dir environment variable is set
      to the directory that MAVBench_base  was cloned to
- TODO: create a set of pre_missions for each app
-- you can use the roslaunch to directory call our applications and interact with them, or you can use
pre_mission.sh to send commands. 
Internal developer Notes: 'c' needs to be pressed always after everything is loaded, so e.g. if it takes a long time for the object detection to get loaded, c needs to be pressed according.

## Host Computer
Some introduction
### System Requirements
**Hardware**:  
+ A system with powerfull CPU + GPU (Our tested setup uses an Intel Core i7 CPU and a high-end NVIDIA GTX 1080 Ti GPU).

**Software**:  
+ Windows 10, 64 bit   (at thist point, we only support windows for the host)
+ Python 2 (also make sure pip is installed)
+ make sure python is in the PATH
+ Visual Studio (tested with visual studio 15.8, 2017 community eddition)  (optional: only if you want to build from scratch)
+ Unreal ( tested with 4.18) (optional: only if you want to build from scratch)


### How to Build 
git clone  --recursive https://github.com/MAVBench/MAVBench.git mavbench_base_dir

**For the lazy yet happy**: We have provided a set of games (environments drone can fly within) that can be simply exectued by the user.
2. cd MAVBench_base/build-scripts
3. host_setup_env_var.cmd 
host_root_setup.cmd

**For the reckless with no life** (most likely you won't fall within this group): In case the user wants to build from scratch (this can be helpful if the user wants to try out new environment maps), follow the insurction provided by Microsoft (https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md) only replacing the **Build AirSim** section with the following instructinos:
1. You will need Visual Studio 2017 (make sure to install VC++ and Windows SDK 8.x).
2. Start x64 Native Tools Command Prompt for VS 2017. 
3.Create a folder for the repo (here on refered to as MAVBench_base) and
5. cd MAVBench_base 
6. host_setup_env_var.cmd 
7. build-scripts/host_root_setup_from_src.cmd
7. This will create ready to use plugin bits in the MAVBench_base/src/AirSim/Unreal/Plugins folder that can be dropped into any Unreal project 
- follow along with the AirSim instuctions provided by Microsoft https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). 
Notes to internal developers: 
      1. If you decided to make your own executable and upload to google drive, use windows to zip and windows to unzip (you can use tar in windows now). I had issues with using tar for unziping.The generated executable was erroring out.
      2. a folder with the name of game/WindowsNoEditor need to be zipped to the google cloud
#### fixing AirSims depth map issue
(*This step may have a problem) Fix a Depth image bug by following this issue: https://github.com/Microsoft/AirSim/issues/491. 
1. Go to BP_PIPCamera (within the unreal editor. This is located under Blueprints (Note: if you can’t find this in the content broweser, click on window->Find in blueprints and serach for BP_RIPCamera)
1. Click on DepthPlannerCaptureComponent (on the left hand side under Components tab). Then in the Details window, click on “post process Materials” and change the material to “DepthMapMaterial”

![alt text](https://github.com/MAVBench/MAVBench/blob/master/docs/images/BP_PIP_depth-map-modification.PNG)


### How to RUN 
**For the lazy yet happy** (method 1):
1. cd MAVBench_base/games/WindowsNoEditor
2. execute the .exe

**For the reckless with no life** (method 2):
ollow the insurction provided by Microsoft (https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). Follow the **How to Use Airsim** Section.  


#### Runnin Hello World

### Profiling
- cd MAVBench_base_dir/build-scripts
- host_setup_env_var.cmd 
- cd MAVBench_base_dir/test_benches/configs
- modify the config file as you like (provide a link here as to the descriptions of the variables in jason file)
- python loader\clct_data.py --config configs\${your_config_file}  (example hellworld-config.json)

#### Interpreting the Results
      the results will be saved in the data/$pkg_name in a jason file

Note: For follow the leader (you can trigger the person (leader) to start moving by pressing r. This time can also be set using
the config file)
## Paper
More technical details are available in our paper published in Micro 2018.(https://d.pr/f/fqspYT);

## Contribute
MAVBench aims at brining the robotics, software and hardware community together. We welcome any contributions such as new kernels, new applications and new hardware setups.

### Games:
Steps to create, upload and deploy games: (for internal developers)
      - make a game in unreal
      - package it
      - zip it (I usually right click->send to->compressed (zipped) folder. I believe 7zip should work too, but not entirely sure)
      - uploaded it to the google drive
      - get a sharable link and paste the id (what's after "id" in the shared link before the next "/") to host_setup_env_var.sh game_fileid variable
## contacts
behzadboro@gmail.com
hngenc@berkeley.edu

## FAQ
### host computer build:
If uppon running host_root_setup.cmd (while in running in "Start x64 Native Tools Command Prompt for VS 2017" shell), you get the following error "The C compiler identification is unknown":
- open up the aformentioned shell in admin mode (right click->Run as an Administrator)
- Proceed to the folder that normaly loads when running "x64 Native Tools Command Prompt for VS 2017" asan user (e.g. C:\Program Files (x86)\Microsoft Visual Studio\2017\Community). start from step 3 again 
Note: that if you want to run as a user, you need to give access to the Community folder (in admin mode) using "icacls "./Community"  /grant $your_user_name:M /T)

Uppon running Unreal, if you get the follow error:

![alt text](https://github.com/MAVBench/MAVBench/blob/master/docs/images/unreal_error.PNG)
- right click the game solution from your solution explorer and select the option "Set as StartUp Project" and that should highlight the game solution. Cntrl-F5 will then work





