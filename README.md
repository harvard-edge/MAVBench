# Welcome to MAVBench 
This README explains how to setup and use MAVBench, A benchmark and a simulator for Micro Aerial Vehicles.


**What is MAVBench?**
MAVBench is a framework targeting design and development of Micro Aerial Vehicles for hardware/software designers and roboticists. It consists of a closed-loop simulator and an end-to-end application
benchmark suite. A closed-loop simulation platform is needed to probe and understand the intra-system (application data flow) and inter-system (system and environment) interactions in MAV applications
to pinpoint bottlenecks and identify opportunities for hardware and software co-design and optimization. In addition to the simulator, MAVBench provides a benchmark suite, the first of its kind,
consisting of a variety of MAV applications designed to enable computer architects to perform characterization and develop future aerial computing systems. This work is built on top of a host of open source software.
A big shout out to Microsoft and ETH Zurich University.

**Why MAVBench?**
We developed MAVBench to accurately model the drone's system and its environment. We identify two main ingredients toward this end.

1. Simulator: Autonomous drones similar to other autonomous machines require a new breed of architectural simulators. Unlike traditional machines (desktops, servers, cellphones and others), information flows in a loop for an autonomous machine . Such flow starts from the machine's environment via sensors, gets processed by the computing subsystem, and flows back out into the environment via actuators.
This means, unlike traditional simulators, autonomous machines require a tightly coupled closed-loop feedback simulator for architectural investigation.   
  Our simulator has three core components as shown in the figure bellow. The drone's environments, sensors, and actuators are simulated using a game engine called Unreal augmented with AirSim libraries (top). By using a physics engine, they provide the ability to simulate the drone's behavior, its environment and the interaction between the two such as accurate collision detection. 
Flight controller (flight stack and the autopilot hardware) is responsible for the drone's stabilization (bottom right). We use a software-simulated flight controller provided by AirSim. However, AirSim also supports other flight controllers, such as the Pixhawk. Much of the drone's perception and trajectory planning is done using an onboard computer, which is generally 
responsible for running any compute-intensive workloads (bottom left). 
We used an NVIDIA Jetson TX2, although our setup allows for swapping this embedded board with other platforms like a RISC-V based platform.   


                                      <img align="center" src="https://github.com/MAVBench/MAVBench/blob/master/docs/images/end-to-end-simulation.png" width="400">                   


2. Benchmark Suite: To quantify the power and performance demands of typical MAV applications, we created a set of workloads that we compiled into a benchmark suite. Our benchmarks run on top of our closed-loop simulation environment. The suite aims to cover a wide range of representative applications. Each workload is an end-to-end application that allows us to study the kernels' impact on the whole application as well as to investigate the interactions and dependencies between kernel. 
  By providing holistic end-to-end applications instead of only focusing on individual kernels, MAVBench allows for the examination of kernels' impacts and their optimization at the application level. This is a lesson learned from Amdahl's law, which recognizes that the true impact of a component's improvement needs to be evaluated globally rather than locally.








## Youtube Channel
[Please visit our youtube channel.](https://www.youtube.com/channel/UC_bNkXcP5BHSRcNJ4R4GTvg)

## System Requirements
### Companion Computer  (Responsible for running the compute intensive workloads)
**Hardware**:  
+ Jetson TX2  

**Software**:  
+ Ubuntu: 16.04  
+ JetPack (NVIDIA SDK): 3.2 (We have only tested our setup with 3.2 but we speculate that it will work with higher versions as well)  

### Host Computer (Responsible for running the drone/environment simulators + autopilot subsystem)

**Hardware**:  
+ A system with powerful CPU + GPU (Our tested setup uses an Intel Core i7 CPU and a high-end NVIDIA GTX 1080 Ti GPU).

**Software**:  
+ Windows 10, 64 bit (at the moment, we only support windows for the host)
+ Python 2 (also make sure pip is installed)
+ Visual Studio (optional: only if you want to build from scratch) (tested with visual studio 15.8, 2017 community edition)  
+ Unreal (optional: only if you want to build from scratch) (tested with 4.18) 




## Building
[Instructions to build MAVBench.](https://github.com/MAVBench/MAVBench/blob/master/docs/readMes/building.md)


## Running 
[Instruction to run MAVBench.](https://github.com/MAVBench/MAVBench/blob/master/docs/readMes/running.md)

## Profiling
[Instruction to profile and interpret the results](https://github.com/MAVBench/MAVBench/blob/master/docs/readMes/building.md)

## Paper
More technical details are available in our paper published in Micro 2018.(https://d.pr/f/fqspYT);

## Contribute
MAVBench aims at brining the robotics, software and hardware community together. We welcome any contributions such as new sensor/actuator models, new kernels/applications and new hardware setups.

## Contributors
Behzad Boroujerdian (UT Austin, Harvard, SiFive)   
Hassan Genc (UC Berkeley)  
Srivatsan Krishnan (Harvard)
Aleksandra Faust (Google Brain)  
Vijay Janapa Reddi  (UT Austin, Harvard, Google)  


## Current Maintainers
behzadboro@gmail.com  
hngenc@berkeley.edu  

## FAQ
