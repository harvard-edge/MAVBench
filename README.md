# Welcome to MAVBench  (Note: We plan to release the code, build scripts and a docker by Micro conference 2018(Oct 20th). As is, the code needs to get cleaned up and the repos might be merged, so please stay tuned)
FORMATTING TODOs: change all the bulles for instructions to numbers.


MAVBench is a framework targetting design and development of Micro Aerial Vehicles for roboticists, sofware and hardware designers. It consists of a closed-loop simulator and an end-to-end application
benchmark suite. A closed-loop simulation platform is needed to probe and understand the intra-system (application data flow) and inter-system (system and environment) interactions in MAV applications
to pinpoint bottlenecks and identify opportunities for hardware and software co-design and optimization. In addition to the simulator, MAVBench provides a benchmark suite, the first of its kind,
consisting of a variety of MAV applications designed to enable computer architects to perform characterization and develop future aerial computing systems. This work is built on top of a host of open
pecially Thanks to Microsoft and University of EHTH zurich

TODO: upload all the videos right here

## Youtube Channel
https://www.youtube.com/channel/UC_bNkXcP5BHSRcNJ4R4GTvg

## Demos

[![Watch the video](https://www.youtube.com/watch?v=B_hOO7o0-Bk)
## Building MAVBench and Using it
closed-loop operation is an integral component of autonomous MAVs. In such systems, the data flows in a (closed) loop, starting from the environment, going through the MAV and back to the environment
. The process involves sensing the environment (Sensors), interpreting it and making decisions (Compute), and finally navigating within or modifying the environment (Actuators) in a loop. 
In our simulator, sensors, actuators, environment and the flight controler are simulated with the help of Unreal and Airsim.
These components need to run on a powerfull machine. Our tested setup uses an Intel Core i7 CPU and a high-end NVIDIA GTX 1080 Ti GPU.   

MAVBench simulator is a closeloop simulator modeling 


## Companion Computer 
### System Requirements
**Hardware**:  
+ Jetson TX2  

**Software**:  
+ Ubuntu: 16.04  
+ JetPack: 3.2 (we have only tested our setup with 3.2 but we suspect, it'll work with higher versions as well)  

### How to Build 
- git clone  --recursive https://github.com/MAVBench/tx2.git MAVBench_base
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
source MAVBench_base/buil-scripts/setup_env_var.sh 
source MAVBench_base/catkin_ws/devel/setup.bash

Note: make sure that mavbench_base_dir environment variable is set
      to the directory that MAVBench_base  was cloned to
- TODO: create a set of pre_missions for each app


## Host Computer
Some introduction
### System Requirements
**Hardware**:  
+ A system with powerfull CPU + GPU (Our tested setup uses an Intel Core i7 CPU and a high-end NVIDIA GTX 1080 Ti GPU).

**Software**:  
+ Windows 10   (at thist point, we only support windows for the host)  
+ Visual Studio (tested with visual studio 15.8, 2017 community eddition)  (optional: only if you want to build from scratch)
+ Unreal ( tested with 4.18) (optional: only if you want to build from scratch)


### How to Build 
**For the lazy yet happy**: We have provided a set of games (environments drone can fly within) that can be simply exectued by the user.
1. git clone  --recursive https://github.com/MAVBench/tx2.git.
2. cd MAVBench_base
3. ./build-scripts/download_games.cmd

**For the reckless with no life**: In case the user wants to build from scratch (this can be helpful if the user wants to try out new environment maps), follow the insurction provided by Microsoft (https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md) only replacing the **Build AirSim** section with the following instructinos:
1. You will need Visual Studio 2017 (make sure to install VC++ and Windows SDK 8.x).
2. Start x64 Native Tools Command Prompt for VS 2017. 
3.Create a folder for the repo (here on refered to as MAVBench_base) and
4. git clone  --recursive https://github.com/MAVBench/tx2.git.
5. cd MAVBench_base 
6. sudo ./build-scripts/host_root_setup.cmd
7. This will create ready to use plugin bits in the MAVBench_base/src/AirSim/Unreal/Plugins folder that can be dropped into any Unreal project 
- follow along with the AirSim instuctions provided by Microsoft https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). 

### How to RUN 
**For the lazy yet happy**:
1. cd MAVBench_base/games
2. execute the binary
**For the reckless with no life**:
ollow the insurction provided by Microsoft (https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md). Follow the **How to Use Airsim** Section.  

### Profiling
- cd MAVBench_base_dir/test_benches
- (somehow they need to set the companion_comp ip)
- python clct_data.py


## Paper
More technical details are available in our paper published in Micro 2018.(https://d.pr/f/fqspYT);

## Contribute
MAVBench aims at brining the robotics, software and hardware community together. We welcome any contributions such as new kernels, new applications and new hardware setups.

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

![alt text](https://github.com/MAVBench/tx2/blob/master/docs/images/unreal_error.PNG)
- right click the game solution from your solution explorer and select the option "Set as StartUp Project" and that should highlight the game solution. Cntrl-F5 will then work





