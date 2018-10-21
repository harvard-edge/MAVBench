# Welcome to MAVBench  (Note: We plan to release the code, build scripts and a docker by Micro conference 2018(Oct 24th). Please STAY TUNED!!!)
This README explains how to setup and use MAVBench.


**What is MAVBench?**
MAVBench is a framework targetting design and development of Micro Aerial Vehicles for roboticists, sofware and hardware designers. It consists of a closed-loop simulator and an end-to-end application
benchmark suite. A closed-loop simulation platform is needed to probe and understand the intra-system (application data flow) and inter-system (system and environment) interactions in MAV applications
to pinpoint bottlenecks and identify opportunities for hardware and software co-design and optimization. In addition to the simulator, MAVBench provides a benchmark suite, the first of its kind,
consisting of a variety of MAV applications designed to enable computer architects to perform characterization and develop future aerial computing systems. This work is built on top of a host of open source software.
A big shout out to Microsoft and University of EHTH zurich. 

**Why MAVBench?**
Autonomous drones similar to other autonomous machines require a new breed of architectural simulators. Unlike in traditional architectural simulators, information in an autonomous machine flows in a loop. Information flows in from the machine's environment via its sensors, gets processed by the computing subsystem, and flows back out into the environment via actuators and controls. Hence, autonomous machines require a tightly coupled closed-loop feedback system for architectural investigation.

We developed MAVBench---a framework consisting of a hardware-in-the-loop simulator and a set of end-to-end benchmarks. To accurately model the drone's system and its environment. Our simulator has three core components (\Fig{fig:end-to-end}). The drone's environments, sensors, and actuators are simulated using a game engine called Unreal augmented with AirSim libraries  (\Fig{fig:end-to-end}, top). By using a physics engine, they provide the ability to simulate the drone's behavior, its environment and the interaction between the two such as accurate collision detection. The flight controller (flight stack and the autopilot hardware) is responsible for the drone's stabilization (\Fig{fig:end-to-end}, bottom right). We use a software-simulated flight controller provided by AirSim. However, AirSim also supports other FCs, such as the Pixhawk. Much of the drone's perception and trajectory planning is done using an onboard computer, which is generally responsible for running any compute-intensive workloads (\Fig{fig:end-to-end}, bottom left). We used an NVIDIA Jetson TX2, although our setup allows for swapping this embedded board with other platforms like a RISC-V based platform. 

![alt text](https://github.com/MAVBench/MAVBench/blob/master/docs/images/end-to-end-simulation.png)


## Youtube Channel
https://www.youtube.com/channel/UC_bNkXcP5BHSRcNJ4R4GTvg

## Building it
[ a link]()

## Running it
[ a link]()

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





