# Welcome to MAVFI
The README explains how to setup MAVFI and conduct fault injection to the applications inside MAVBench, a benchmark and a simulator for Micro Aerial Vehicles. 


**What is MAVFI**
To understand the fault tolerance and end-to-end reliability impact of transient faults in MAVs, we built a fault injection framework for MAV applications, referred to as MAVFI. The MAVFI is a ROS node that can be integrated into ROS-based MAV applications and injects transient faults to each compute node in the system, facilitating fault tolerance analysis for diverse MAV applications. 

MAVFI is able to conduct architecture-level fault injection for CPU by introducing single or multiple bit flips in the register being accessed by current instruction, simulating transient faults happened during the execution of the processor. The transient faults injected to the architectural states can be manifested and cause the data corruption of output variables. To the best of our knowledge, MAVFI is the first fault injection framework targeted at ROS-based applications. MAVFI can be integrated into different applications and perform fault injection at close to the native speed of the original end-to-end system. In general, MAVFI facilitates fault tolerance analysis of any ROS-based applications, beyond just MAVs. 

**What is MAVBench?**
MAVBench is a framework targeting design and development of Micro Aerial Vehicles for hardware/software designers and roboticists. It consists of a closed-loop simulator and an end-to-end application
benchmark suite. A closed-loop simulation platform is needed to probe and understand the intra-system (application data flow) and inter-system (system and environment) interactions in MAV applications
to pinpoint bottlenecks and identify opportunities for hardware and software co-design and optimization. In addition to the simulator, MAVBench provides a benchmark suite, the first of its kind,
consisting of a variety of MAV applications designed to enable computer architects to perform characterization and develop future aerial computing systems. 

<img align="right" src="https://github.com/MAVBench/MAVBench/blob/master/docs/images/end_to_end_simulation.png" width="500">  
1. Simulator: Autonomous drones similar to other autonomous machines require a new breed of architectural simulators. Unlike traditional machines (desktops, servers, cellphones and others), information flows in a loop for an autonomous machine . Such flow starts from the machine's environment via sensors, gets processed by the computing subsystem, and flows back out into the environment via actuators.
This means, unlike traditional simulators, autonomous machines require a tightly coupled closed-loop feedback simulator for architectural investigation.   
  The MAVBench simulator has three core components as shown in the figure bellow. The drone's environments, sensors, and actuators are simulated using a game engine called Unreal augmented with AirSim libraries (top). By using a physics engine, they provide the ability to simulate the drone's behavior, its environment and the interaction between the two such as accurate collision detection. 
Flight controller (flight stack and the autopilot hardware) is responsible for the drone's stabilization (bottom right). MAVBench use a software-simulated flight controller provided by AirSim. However, AirSim also supports other flight controllers, such as the Pixhawk. Much of the drone's perception and trajectory planning is done using an onboard computer, which is generally 
responsible for running any compute-intensive workloads (bottom left). 
MAVBench used an NVIDIA Jetson TX2, although the setup allows for swapping this embedded board with other platforms like a RISC-V based platform.   


<img align="right" src="https://github.com/MAVBench/MAVBench/blob/master/docs/images/suite_vertical.png" width="500">
2. Benchmark Suite: To quantify the power and performance demands of typical MAV applications, MAVBench created a set of workloads that we compiled into a benchmark suite. Our benchmarks run on top of the closed-loop simulation environment. The suite aims to cover a wide range of representative applications. Each workload is an end-to-end application that allows us to study the kernels' impact on the whole application as well as to investigate the interactions and dependencies between kernel. 
  By providing holistic end-to-end applications instead of only focusing on individual kernels, MAVBench allows for the examination of kernels' impacts and their optimization at the application level. This is a lesson learned from Amdahl's law, which recognizes that the true impact of a component's improvement needs to be evaluated globally rather than locally.





## Building
MAVFI has already be integrated into MAVBench. Please follow the same intruction as MAVBench to build MAVFI. (docs/read_me/building.md)


## Running 
MAVFI supports single or multiple bit flips to the ROS node of package delivery application in MAVBench. User can define the number of bit flips and which algorithm (e.g., Octomap, motion planner) to inject fault into inside the launch file of package delivery application (src/MAV_apps/package_delivery/launch/package_delivery.launch). After configuration, please follow the instruction in docs/read_me/running.md to run MAVBench with MAVFI.


## Directory Structure
```bash
.
├── build_scripts # Scripts for building our repo and subrepos
├── docs          # Documents
│   ├── images    
│   └── read_me   
├── src           # All the src code
│   ├── AirSim
│   ├── darknet
│   ├── mav-bench-apps
│   ├── opencv
│   └── pcl
└── test_benches  # Test benches allowing the user to 1.use pre-defined missions
                  #                                  2. profile
    ├── configs   # Pre-defined missions (you can change this according to your
   		    need)
    └── scripts   # Scripts to load test benches and profile
```


