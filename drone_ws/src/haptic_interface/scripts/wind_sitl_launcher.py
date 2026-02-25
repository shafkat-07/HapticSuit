#!/usr/bin/env python3

import rospy
import subprocess
import os
import sys

def launch_sitl():
    """
    Launches the ArduPilot SITL (sim_vehicle.py) with explicit output for MAVROS.
    Tries to use xterm for console, falls back to direct execution if xterm fails.
    """
    rospy.init_node('wind_sitl_launcher', anonymous=True)
    rospy.loginfo("Wind SITL Launcher: Starting sim_vehicle.py...")

    home = os.path.expanduser('~')
    sitl_directory = os.path.join(home, 'ardupilot', 'ArduCopter')
    sim_vehicle_path = os.path.join(home, 'ardupilot', 'Tools', 'autotest', 'sim_vehicle.py')

    # Explicitly add output for MAVROS (UDP 14550)
    # Note: sim_vehicle.py usually adds 14550 by default, but we are explicit to avoid ambiguity
    # Use --no-mavproxy to avoid the MAVProxy spam if just being used as a bridge, 
    # BUT we need MAVProxy to create the UDP bridge to MAVROS (14550).
    # The spam is coming from MAVProxy's standard output.
    base_cmd = [
        'python3',
        sim_vehicle_path,
        '-v', 'ArduCopter',
        '-f', 'gazebo-iris',
        '--console',
        '--out=udp:127.0.0.1:14550' 
    ]

    # Try to launch in xterm first (standard behavior)
    xterm_cmd = ['xterm', '-hold', '-e'] + [" ".join(base_cmd)]
    
    # Check if xterm is available and we have a display
    has_xterm = False
    try:
        subprocess.check_call(['which', 'xterm'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        has_xterm = True
    except subprocess.CalledProcessError:
        pass

    display = os.environ.get('DISPLAY')
    
    try:
        if has_xterm and display:
            rospy.loginfo("Launching SITL in xterm...")
            # We use cwd to ensure ArduCopter context
            process = subprocess.Popen(xterm_cmd, cwd=sitl_directory)
            rospy.loginfo(f"SITL (xterm) PID: {process.pid}")
            process.wait()
        else:
            rospy.logwarn("xterm not found or no DISPLAY. Launching SITL directly (no separate console window).")
            # Run directly without xterm. Output will be in ROS log.
            # We might need to remove --console if it requires a TTY, but sim_vehicle usually handles it.
            # However, --console tries to launch MAVProxy with GUI.
            # If headless, we should probably use --no-mavproxy or --map?
            # But we need MAVProxy for the connection (UDP 14550).
            # MAVProxy can run without GUI (--daemon or --nowait?).
            # sim_vehicle.py passes args to MAVProxy.
            
            # For robustness in headless, we just run it. 
            # If MAVProxy fails to open a window, it might still run.
            process = subprocess.Popen(base_cmd, cwd=sitl_directory)
            rospy.loginfo(f"SITL (direct) PID: {process.pid}")
            process.wait()

    except Exception as e:
        rospy.logerr(f"SITL Launcher failed: {e}")

if __name__ == '__main__':
    try:
        launch_sitl()
    except rospy.ROSInterruptException:
        pass
