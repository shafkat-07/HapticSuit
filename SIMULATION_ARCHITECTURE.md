# Haptic Suit Drone Simulation Project

## 1. Project Overview

This project aims to create a high-fidelity, hardware-in-the-loop simulation environment for testing and validating drone dynamics using a custom haptic suit. The core objective is to stream the physical state of a simulated F450 drone from the Gazebo simulator to a physical haptic suit. The suit will then mimic the forces and orientation of the drone, providing tangible feedback to the user and serving as a platform for testing control algorithms.

## 2. Software Architecture

The system is composed of four primary components that work together to create the full simulation loop.

![Architecture Diagram](https://i.imgur.com/your-diagram-link.png)  <!-- You can create and upload a diagram to a service like imgur.com -->

### 2.1. Gazebo Simulation (`drone_ws`)
- **Role:** Simulates the physical world, drone dynamics, and sensors.
- **Drone Model:** A custom F450 drone model (`model.sdf`) is used. This model, including its visual meshes and physical properties, was sourced from the *Autonomous Landing UAV* project. This work was published in the paper "Monocular Visual Autonomous Landing System for Quadcopter Drones Using Software in the Loop".
- **Plugins:** The F450 model relies on the `rotors_simulator` Gazebo plugins to simulate on-board sensors (IMU, GPS, barometer, magnetometer) and to provide the crucial MAVLink communication bridge to the autopilot.

### 2.2. Autopilot (ArduPilot SITL)
- **Role:** Runs the complete ArduPilot flight control software in a Software-In-The-Loop (SITL) configuration.
- **Process:** The `sim_vehicle.py` script launches the ArduPilot binary. It receives simulated sensor data from Gazebo and computes the necessary motor outputs to achieve stable flight, exactly as it would on the real hardware. This provides a very realistic simulation of the drone's brain.

### 2.3. ROS (Robot Operating System)
- **Role:** Acts as the central nervous system, connecting all software components and enabling communication between them.
- **MAVROS (`mavros` node):** The official MAVLink-to-ROS bridge. It translates MAVLink messages from ArduPilot into standard ROS topics and services, and vice-versa. This is the primary way we interact with the simulated drone.
- **Control & Data Nodes:**
    - `f450_keyboard_teleop.py`: A temporary node for manually testing the simulation by sending velocity commands to MAVROS.
    - `gazebo_to_haptic_bridge.py` (Future): This future node will be the link between the simulation and the physical suit.

### 2.4. Haptic Suit Controller (`HapticSuitController.ino`)
- **Role:** The physical hardware that receives data from the simulation and actuates motors to provide haptic feedback.
- **Hardware:** An Arduino-based controller with a WiFi module.
- **Communication:** It listens for UDP packets on port `8888`. These packets contain comma-separated setpoint values (e.g., roll, motor throttle) for the suit's arms.
- **Control:** It runs an onboard PID control loop to drive its motors to match the setpoints received from the simulation.

## 3. References & Citation

If you use the F450 model from this work, please cite the original paper:
```
@article{9656574,
	title        = {Monocular Visual Autonomous Landing System for Quadcopter Drones Using Software in the Loop},
	author       = {Saavedra-Ruiz, Miguel and Pinto-Vargas, Ana Maria and Romero-Cano, Victor},
	year         = 2022,
	journal      = {IEEE Aerospace and Electronic Systems Magazine},
	volume       = 37,
	number       = 5,
	pages        = {2--16},
	doi          = {10.1109/MAES.2021.3115208}
}
```

## 4. Data Flow

The project is broken into two main phases, each with a distinct data flow.

### Phase 1: Simulation Control (Current Step)
*This phase focuses on ensuring the simulation is stable and controllable.*
1.  **Input:** The `f450_keyboard_teleop.py` node captures keyboard presses.
2.  **ROS Command:** It publishes `geometry_msgs/Twist` messages to the `/mavros/setpoint_velocity/cmd_vel_unstamped` topic.
3.  **Translation:** The `mavros` node converts this ROS message into a `MAVLink` command.
4.  **Autopilot:** The MAVLink command is sent to the `ArduPilot SITL`, which calculates the required motor responses.
5.  **Actuation:** ArduPilot sends MAVLink motor commands back to the `mavlink_interface` Gazebo plugin.
6.  **Simulation:** The plugin applies forces to the simulated motors in Gazebo, and the physics engine calculates the drone's new state.
7.  **Sensor Feedback:** The simulated sensors generate new data, which is sent back to ArduPilot, closing the control loop.

### Phase 2: Haptic Data Streaming (Next Step)
*This phase bridges the simulation with the physical hardware.*
1.  **Data Subscription:** The new `gazebo_to_haptic_bridge.py` node will subscribe to ROS topics published by MAVROS, such as `/mavros/imu/data` (for orientation and forces) and `/mavros/state`.
2.  **Data Processing:** The node will extract the relevant information (e.g., roll, pitch, linear acceleration).
3.  **Formatting:** It will format this data into the comma-separated string that the Haptic Suit Controller expects.
4.  **UDP Transmission:** The node will send the formatted string as a UDP packet to the Haptic Suit's IP address on port `8888`.
5.  **Haptic Feedback:** The Haptic Suit receives the packet and actuates its motors to physically represent the drone's state.

## 5. Current Status & Next Steps

We are currently in the final stages of **Phase 1**. The simulation environment, ArduPilot SITL, and MAVROS are all configured. The final blocker is a missing dependency required by the custom F450 Gazebo model.

### Immediate Next Steps:
1.  **Install Dependencies:** Install the `rotors_simulator` plugins to enable the F450's sensors in Gazebo.
    ```bash
    sudo apt-get install ros-noetic-rotors-simulator-gazebo-plugins
    ```
2.  **Verify Keyboard Control:** Relaunch the entire simulation stack and confirm that the F450 drone can be successfully armed, taken off, and controlled in Gazebo via the keyboard.

### Phase 2 Plan:
3.  **Create Bridge Node:** Create the `gazebo_to_haptic_bridge.py` ROS node.
4.  **Implement ROS Subscription:** Write the code to subscribe to the necessary MAVROS topics.
5.  **Implement UDP Communication:** Add the logic to format and send the data string over UDP to the haptic suit.
6.  **End-to-End Test:** Run the full system and validate that the haptic suit responds correctly to the drone's movements in the simulation.

## 6. How to Run the Full Simulation
*Execute each command in a separate terminal.*

1.  **Launch Gazebo:**
    ```bash
    roslaunch flight_controller f450_sitl.launch
    ```
2.  **Start ArduPilot SITL:**
    ```bash
    sim_vehicle.py -v ArduCopter -f gazebo-iris --console
    ```
3.  **Start MAVROS:**
    ```bash
    roslaunch flight_controller mavros_sitl.launch
    ```
4.  **Start Keyboard Controller:**
    ```bash
    rosrun flight_controller f450_keyboard_teleop.py
    ```
