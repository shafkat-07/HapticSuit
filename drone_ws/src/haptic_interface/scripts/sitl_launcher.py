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

    # The sim_vehicle.py script must be run from its own directory.
    sitl_directory = os.path.expanduser('~/ardupilot/ArduCopter')
    
    # We use Popen to launch the process in a new terminal window.
    # The command must be broken into a list of arguments.
    # Using xterm is a common way to launch a visible terminal from a ROS launch file.
    command = [
        'xterm',
        '-e',
        './sim_vehicle.py -v ArduCopter -f gazebo-iris --console'
    ]

    try:
        # We use cwd to change the current working directory to where the script needs to run.
        # This is the key to making this work reliably.
        process = subprocess.Popen(command, cwd=sitl_directory)
        rospy.loginfo(f"SITL Launcher: sim_vehicle.py launched in directory {sitl_directory} with PID: {process.pid}")
        
        # We can wait for the process to complete, or just let it run.
        # Since this is a long-running process, we'll let it run and the node will exit.
        # process.wait() 
        
    except FileNotFoundError:
        rospy.logerr(f"SITL Launcher Error: The command 'xterm' was not found. Please ensure it is installed (`sudo apt-get install xterm`).")
    except Exception as e:
        rospy.logerr(f"SITL Launcher failed: {e}")

if __name__ == '__main__':
    try:
        launch_sitl()
    except rospy.ROSInterruptException:
        pass
