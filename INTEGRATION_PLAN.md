# **World-in-the-Loop Haptic Suit: Clean Slate Implementation Plan**

## **1\. The Strategy**

We are abandoning the custom "game-like" simulation (which lacked physics) and moving to the **Industry Standard Stack**.

* **Physics:** Gazebo (Calculates wind, gravity, collisions).  
* **Flight Control:** ArduPilot SITL (The actual C++ flight code, running on your PC).  
* **Bridge:** ROS Noetic \+ MAVROS (Extracts the data).  
* **Your Code:** A single script (haptic\_bridge.py) that sends force data to the suit.

## **2\. Phase 1: Installation & Setup**

*Goal: Get the standard simulation running before writing any custom code.*

### **Step 1: Install ROS Dependencies**

Open a terminal and run these commands to install the communication bridges.

```
sudo apt-get update  
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-gazebo-ros-pkgs  
wget \[https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install\_geographiclib.sh\](https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install\_geographiclib.sh)  
sudo bash ./install\_geographiclib.sh
```

### **Step 2: Install ArduPilot SITL**

This compiles the flight controller code for your laptop.
```
cd \~  
git clone \--recurse-submodules \[https://github.com/ArduPilot/ardupilot.git\](https://github.com/ArduPilot/ardupilot.git)  
cd ardupilot  
Tools/environment\_install/install-prereqs-ubuntu.sh \-y  
. \~/.profile
```

### **Step 3: Install the Gazebo Plugin**

This is the critical "missing link" that connects Gazebo physics to ArduPilot.
```
cd \~  
git clone \[https://github.com/ArduPilot/ardupilot\_gazebo\](https://github.com/ArduPilot/ardupilot\_gazebo)  
cd ardupilot\_gazebo  
mkdir build && cd build  
cmake ..  
make \-j4  
sudo make install
```
### **Step 4: Configure Paths**

Add the model paths to your environment so Gazebo can find the drone.
```
echo 'export GAZEBO\_MODEL\_PATH=\~/ardupilot\_gazebo/models:${GAZEBO\_MODEL\_PATH}' \>\> \~/.bashrc  
source \~/.bashrc
```
### **Step 4: Build the ROS Workspace**

Before running the simulation, you need to build your ROS workspace to recognize the new `haptic_interface` package and its launch file.
```
cd drone_ws
catkin_make
source devel/setup.bash
```

## **3\. Phase 2: Running the Simulation**

*Goal: Launch a drone that is controlled by real flight logic.*

Thanks to the new ROS launch file, the entire simulation can be started with a single command. This will automatically open Gazebo, the ArduPilot flight controller (in a new terminal window), MAVROS, and the haptic bridge.

```
roslaunch haptic_interface start_simulation.launch
```

* **Success Check:** 
    * You should see the drone in a Gazebo world.
    * A new terminal window should pop up showing the MAVProxy console.
    * Run `rostopic echo /mavros/state` in a new terminal and look for `connected: True`.

## **4\. Phase 3: The Haptic Bridge Code**

*Goal: Extract the "Force" data and send it to your suit.*

The launch file automatically runs the `haptic_bridge.py` script for you. The code is located in `drone_ws/src/haptic_interface/scripts/haptic_bridge.py`.

```
#!/usr/bin/env python3  
import rospy  
import socket  
import time  
from sensor\_msgs.msg import Imu

\# \--- CONFIGURATION \---  
\# IP Address of the ESP32 on your Haptic Suit  
SUIT\_IP \= "192.168.1.100"   
SUIT\_PORT \= 8888  
\# Mass of the drone in Gazebo (Iris is approx 1.5kg)  
DRONE\_MASS \= 1.5 

class HapticBridge:  
    def \_\_init\_\_(self):  
        rospy.init\_node('haptic\_bridge')  
          
        \# 1\. Setup UDP Connection to Suit  
        self.sock \= socket.socket(socket.AF\_INET, socket.SOCK\_DGRAM)  
          
        \# 2\. Subscribe to IMU Data  
        \# /mavros/imu/data contains the raw acceleration felt by the drone  
        self.sub\_imu \= rospy.Subscriber('/mavros/imu/data', Imu, self.imu\_cb)  
          
        rospy.loginfo(f"Haptic Bridge Initialized. Sending to {SUIT\_IP}:{SUIT\_PORT}")

    def imu\_cb(self, msg):  
        """  
        Runs \~50 times per second.   
        Reads simulated physics \-\> Sends commands to physical suit.  
        """  
        \# A. Read Acceleration from Gazebo (m/s^2)  
        ax \= msg.linear\_acceleration.x  
        ay \= msg.linear\_acceleration.y  
        az \= msg.linear\_acceleration.z  
          
        \# B. Calculate Force (F \= ma)  
        \# Note: We subtract 9.81 from Z to remove gravity's constant pull  
        \# for a "deviation" based force.  
        force\_x \= ax \* DRONE\_MASS  
        force\_y \= ay \* DRONE\_MASS  
        force\_z \= (az \- 9.81) \* DRONE\_MASS 

        \# C. Simple Noise Filter (Deadzone)  
        \# Ignores tiny jitters in the simulation  
        if abs(force\_x) \< 0.2: force\_x \= 0  
        if abs(force\_y) \< 0.2: force\_y \= 0  
        if abs(force\_z) \< 0.2: force\_z \= 0

        \# D. Format Packet  
        \# Format: "fx,fy,fz" (e.g., "1.2,-0.5,0.0")  
        packet \= f"{force\_x:.2f},{force\_y:.2f},{force\_z:.2f}"  
          
        \# E. Send to Suit  
        try:  
            self.sock.sendto(packet.encode(), (SUIT\_IP, SUIT\_PORT))  
              
            \# Debugging: Only print if there is significant wind/force  
            if abs(force\_x) \> 1.0 or abs(force\_y) \> 1.0:  
                rospy.loginfo(f"TURBULENCE DETECTED\! Force: {packet}")  
                  
        except Exception as e:  
            rospy.logwarn(f"UDP Error: {e}")

if \_\_name\_\_ \== '\_\_main\_\_':  
    try:  
        bridge \= HapticBridge()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        pass
```
## **5\. Phase 4: Validation (How to prove it works)**

You don't need to fly the drone manually to test this.

1. **Launch the simulation:** Run `roslaunch haptic_interface start_simulation.launch`.
2. **Inject Wind:** Go to the MAVProxy console (the separate terminal that opened) and type:  
   `param set SIM_WIND_SPD 15`  
   `param set SIM_WIND_DIR 90`

3. **The Result:** * The simulated drone will tilt to fight the wind.  
   * The IMU will register this acceleration.  
   * Your script will print "TURBULENCE DETECTED\!" and send the data to the suit.  
   * **Your Suit should respond physically.**