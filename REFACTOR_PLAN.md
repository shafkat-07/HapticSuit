# Haptic Suit Simulation: Refactor & Packaging Plan

This document outlines the plan to transform the current development setup into a robust, configurable, and user-friendly simulation package. The goal is to move from a multi-step manual process to a single, flexible launch command.

---

### **Phase 1: Robust Automation (The Immediate Fix)**

The primary goal of this phase is to fix the current launch process, ensuring that all components (Gazebo, SITL, MAVROS, Haptic Bridge) start reliably with a single command.

1.  **Replace Shell Script with a Python Launcher:**
    *   The existing `run_sitl.sh` script, which is fragile, will be replaced with a new Python script named `sitl_launcher.py`.
    *   This script will be a proper ROS node, providing better integration with the ROS ecosystem.

2.  **Use Python's `subprocess` Module:**
    *   The new Python launcher will use the `subprocess` module to execute the `sim_vehicle.py` script.
    *   This method gives us fine-grained control over the execution environment, ensuring the script runs in the correct directory (`~/ardupilot/ArduCopter`) and inherits the necessary shell environment.

3.  **Update the Main Launch File:**
    *   The `start_simulation.launch` file will be modified to execute the new `sitl_launcher.py` node instead of the old shell script.

*   **Outcome:** The command `roslaunch haptic_interface start_simulation.launch` will reliably start the entire simulation stack, including the MAVProxy console, without any manual intervention.

---

### **Phase 2: Configurability for Scenarios (The Core Goal)**

This phase focuses on making the simulation flexible, allowing users to select different scenarios (worlds, conditions) easily from the command line.

1.  **Introduce ROS Launch Arguments:**
    *   The `start_simulation.launch` file will be updated to use `<arg>` tags. These act as variables that can be defined at launch time.

2.  **Create a Selectable World Argument:**
    *   The first argument will be `world_name`, which will default to the `iris_arducopter_runway.world`.
    *   Users will be able to launch a custom world easily with a command like:
        ```bash
        roslaunch haptic_interface start_simulation.launch world_name:=/path/to/my_custom_world.world
        ```

3.  **Develop a Library of Scenarios:**
    *   A new directory, `launch/scenarios/`, will be created.
    *   This directory will contain simple launch files for pre-configured scenarios (e.g., `high_wind.launch`, `obstacle_course.launch`).
    *   These scenario files will simply call the main `start_simulation.launch` file with different arguments, making it easy to expand the list of available tests.

*   **Outcome:** The simulation will be highly configurable, allowing for rapid testing of different environments and conditions without modifying any core files.

---

### **Phase 3: Creating a True Software Package (The Long-Term Vision)**

This final phase polishes the project into a distributable package that is easy for new users to install and run.

1.  **Automate Scenario Conditions:**
    *   Wind parameters and other environmental conditions will be added as launch file arguments.
    *   A new ROS node can be created to listen for these parameters and automatically send the corresponding commands to the MAVProxy console on startup.
    *   Obstacles and other dynamic elements can be spawned into Gazebo directly from the launch file.

2.  **Write Comprehensive Documentation:**
    *   A master `README.md` file will be created at the project root.
    *   It will provide a clear "menu" of available launch scenarios, explain all configurable arguments, and give clear instructions on how to use the package.

3.  **Create a Master Installation Script:**
    *   A single `install_dependencies.sh` script will be written to automate all the setup steps (installing ROS, ArduPilot, Gazebo plugins, etc.).
    *   This will reduce the complex, multi-step installation process to a single command, making it incredibly easy for a new user to get started.

*   **Outcome:** The project will be a self-contained, well-documented, and easily installable software package, fulfilling the vision of a shareable haptic simulation environment.
