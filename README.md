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
[Please visit our youtube channel.](https://www.youtube.com/channel/UC_bNkXcP5BHSRcNJ4R4GTvg)

## System Requirements
### Companion Computer  (Responsible for running the compute intensive workloads)
**Hardware**:  
+ Jetson TX2  

**Software**:  
+ Ubuntu: 16.04  
+ JetPack: 3.2 (We have only tested our setup with 3.2 but we suspect, it'll work with higher versions as well)  

### Host Computer (Responsible for running the drone/environment simualtors + autopilot subsystem)

**Hardware**:  
+ A system with powerfull CPU + GPU (Our tested setup uses an Intel Core i7 CPU and a high-end NVIDIA GTX 1080 Ti GPU).

**Software**:  
+ Windows 10, 64 bit   (at thist point, we only support windows for the host)
+ Python 2 (also make sure pip is installed)
+ Visual Studio (optional: only if you want to build from scratch) (tested with visual studio 15.8, 2017 community eddition)  
+ Unreal (optional: only if you want to build from scratch) ( tested with 4.18) 




## Building
[Instructions to build MAVBench.](https://github.com/MAVBench/MAVBench/blob/master/docs/readMes/building.md)


## Running 
[Instruction to run MAVBench.](https://github.com/MAVBench/MAVBench/blob/master/docs/readMes/running.md)

## Profiling
[Instruction to profile and interprete the results](https://github.com/MAVBench/MAVBench/blob/master/docs/readMes/building.md)

## Paper
More technical details are available in our paper published in Micro 2018.(https://d.pr/f/fqspYT);

## Contribute
MAVBench aims at brining the robotics, software and hardware community together. We welcome any contributions such as new kernels, new applications and new hardware setups.

## contacts
behzadboro@gmail.com
hngenc@berkeley.edu

## FAQ




