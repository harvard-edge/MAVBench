# Welcome to MAVBench  (Note: We plan to release the code, build scripts and a docker by Micro conference 2018(Oct 20th). As is, the code needs to get cleaned up and the repos might be merged, so please stay tuned)
MAVBench is a framework targetting design and development of Micro Aerial Vehicles for roboticists, sofware and hardware designers. It consists of a closed-loop simulator and an end-to-end application
benchmark suite. A closed-loop simulation platform is needed to probe and understand the intra-system (application data flow) and inter-system (system and environment) interactions in MAV applications
to pinpoint bottlenecks and identify opportunities for hardware and software co-design and optimization. In addition to the simulator, MAVBench provides a benchmark suite, the first of its kind,
consisting of a variety of MAV applications designed to enable computer architects to perform characterization and develop future aerial computing systems. This work is built on top of a host of open
pecially Thanks to Microsoft and University of EHTH zurich

## Youtube Channel
https://www.youtube.com/channel/UC_bNkXcP5BHSRcNJ4R4GTvg

## Building MAVBench and Using it

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
- sudo ./build-scripts/root_setup.bash 
- ./build-scripts/user_setup.bash

#### build notes:
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
## Paper
More technical details are available in our paper published in Micro 2018.(https://d.pr/f/fqspYT);

## Contribute
MAVBench aims at brining the robotics, software and hardware community together. We welcome any contributions such as new kernels, new applications and new hardware setups.

## contacts
behzadboro@gmail.com
hngenc@berkeley.edu

## FAQ





