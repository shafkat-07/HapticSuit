#!/usr/bin/env python3

import rospy
import subprocess
import os

def launch_sitl():
    """
    Launches the ArduPilot SITL (sim_vehicle.py).
    This script is a robust replacement for the run_sitl.sh script,
    using Python's subprocess module for better control and error handling.
    """
    rospy.init_node('sitl_launcher', anonymous=True)
    rospy.loginfo("SITL Launcher: Starting sim_vehicle.py...")

    home = os.path.expanduser('~')
    sitl_directory = os.path.join(home, 'ardupilot', 'ArduCopter')
    sim_vehicle_path = os.path.join(home, 'ardupilot', 'Tools', 'autotest', 'sim_vehicle.py')

    command = [
        'xterm',
        '-hold',
        '-e',
        f'python3 {sim_vehicle_path} -v ArduCopter -f gazebo-iris --console'
    ]

    try:
        # We use cwd to change the current working directory to where the script needs to run.
        # This is the key to making this work reliably.
        process = subprocess.Popen(command, cwd=sitl_directory)
        rospy.loginfo(f"SITL Launcher: sim_vehicle.py launched in directory {sitl_directory} with PID: {process.pid}")
        
        # We can wait for the process to complete, or just let it run.
        # Since this is a long-running process, we'll let it run and the node will exit.
        process.wait() 
        
    except FileNotFoundError:
        rospy.logerr(f"SITL Launcher Error: The command 'xterm' was not found. Please ensure it is installed (`sudo apt-get install xterm`).")
    except Exception as e:
        rospy.logerr(f"SITL Launcher failed: {e}")

if __name__ == '__main__':
    try:
        launch_sitl()
    except rospy.ROSInterruptException:
        pass
