# Mimicking Real Forces on a Drone Through a Haptic Suit to Enable Cost-Effective Validation

Robots operate under certain forces that affect their behavior. For example, a drone meant to deliver packages must hold its pose as long as it operates under its weight and wind limits. Validating that such a drone handles external forces correctly is key to ensuring its safety. Nevertheless, validating the system's behavior under the effect of such forces can be difficult and costly. For example, checking the effects of different wind magnitudes may require waiting for the matching outdoor conditions or building specialized devices like wind tunnels. Checking the effects of different package sizes and shapes may require many slow and laborious iterations, and validating the combinations of wind gusts and package configurations is often hard to replicate. This work introduces a framework to overcome such challenges by mimicking external forces exercised on a drone with limited cost, setup, and space. The framework consists of a haptic suit device consisting of directional propellers that can be mounted onto a drone, a component to transform intended forces into setpoints for the suit's directional propellers, and a controller for the suit to meet those setpoints. We conduct a study to assess the framework's capabilities under multiple scenarios involving various forces. Our findings show that the haptic suit framework can recreate real-world forces on the drone with acceptable precision.

[![Video](./misc/cover.png)](https://youtu.be/5_QmRLWMhes)

## Prerequisites

### Hardware

This software was developed, run and test on the following machines. 

| Computer   | CPU                              | CPU Cores | RAM       | Operating System  |
|------------|------------------------------	|-------	|-------	|---------------    |
| Computer 1 | AMD Ryzen Threadripper 3990X     | 128       | 128 GB    | Ubuntu 20.04      |
| Computer 2 | Intel Core i7-10750H             | 12        | 16 GB     | Ubuntu 20.04      |

### Software

We require that you have [ROS Noetic](http://wiki.ros.org/noetic/Installation) to operate the drone. We require that you use the [Arduino IDE](https://www.arduino.cc/en/software) for flashing the haptic suit controllers. Finally we require that you have [SOLIDWORKS](https://www.solidworks.com) and a 3D printer for printing the arms of the haptic suit. Finally we require that you have [Matlab](https://www.mathworks.com/products/matlab.html) for computing the inverse kinematics.

## Getting Started

| Component         	| Description                                                                                               	| Link 	                                    |
|-------------------	|-----------------------------------------------------------------------------------------------------------	|----------------------------------------   |
| Kinematics       	    | This will describe how we implemented the kinematics and includes Matlab code used to solve the inverse.    	| [README.MD](./kinematics/README.md)    	|
| Physical Design       | This includes the designs for the physical arms, battery holders, and pendulum used in our study.            	| [README.MD](./physical_design/README.md)  |
| Drone Workspace      	| This has the software used to fly the drone in each of the scenarios listed in the paper.                   	| [README.MD](./drone_ws/README.md)         |
| Firmware            	| This describes how to flash the firmware used by the haptic suit                                              | [README.MD](./firmware/README.md)    	    |
| Paper            	    | Includes the paper published at [ICRA 2023](https://www.icra2023.org)                               | [Paper.pdf](./paper/HapticSuit.pdf)    	|
