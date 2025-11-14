# F450 Drone Simulation

Complete F450 quadcopter simulation with realistic flight dynamics, sensor suite, and keyboard control for the Haptic Suit project.

## Features

- ✅ **Hector Quadrotor Controller**: Professional-grade quadrotor flight dynamics
- ✅ **Realistic GPS**: Proper GPS simulation with noise, drift, and velocity estimation
- ✅ **Full Sensor Suite**: IMU, Camera, Ground Truth Odometry, GPS
- ✅ **Keyboard Control**: Intuitive WASD + IJK controls
- ✅ **Comprehensive State Publishing**: All sensor data aggregated on clean ROS topics
- ✅ **No Flight Controller Dependencies**: Pure ROS/Gazebo (no PX4/Ardupilot required)

## Prerequisites

### Required

```bash
# ROS Noetic (should already be installed)
# Gazebo 11 (comes with ROS Noetic)

# Hector Gazebo Plugins (ONE-TIME INSTALL)
sudo apt-get update
sudo apt-get install -y ros-noetic-hector-gazebo-plugins
```

### Build

```bash
cd ~/source/HapticSuit/drone_ws
catkin_make
source devel/setup.bash
```

## Quick Start

### 1. Launch the Simulation

```bash
roslaunch flight_controller f450_simulation.launch
```

This starts:
- Gazebo with F450 drone
- Hector quadrotor controller
- State publisher (aggregates all sensor data)
- Motor enable publisher

### 2. Control the Drone

**Option A: Keyboard Control (Recommended)**

In a separate terminal:
```bash
roslaunch flight_controller keyboard_teleop.launch
```

**Controls:**
- `W`/`S` - Forward/Backward
- `A`/`D` - Left/Right
- `I`/`K` - Up/Down
- `J`/`L` - Rotate Left/Right
- `Q`/`Z` - Increase/Decrease all speeds
- `T` - Takeoff
- `G` - Land
- `SPACE` - Stop (hover)
- `CTRL+C` - Quit

**Option B: Manual Topic Control**

```bash
# Takeoff (go up)
rostopic pub /f450/cmd_vel geometry_msgs/Twist "linear: {x: 0, y: 0, z: 0.5}" -r 10

# Move forward
rostopic pub /f450/cmd_vel geometry_msgs/Twist "linear: {x: 1.0, y: 0, z: 0}" -r 10

# Hover (stop)
rostopic pub /f450/cmd_vel geometry_msgs/Twist "linear: {x: 0, y: 0, z: 0}" -r 10
```

## Published Topics

### Raw Sensor Data
- `/f450/ground_truth/odom` (nav_msgs/Odometry) - Ground truth pose and velocity at 50Hz
- `/f450/imu/data` (sensor_msgs/Imu) - IMU readings at 100Hz
- `/f450/gps/fix` (sensor_msgs/NavSatFix) - GPS coordinates with realistic noise at 10Hz
- `/f450/gps/fix_velocity` (geometry_msgs/Vector3Stamped) - GPS velocity estimates
- `/f450/camera/image_raw` (sensor_msgs/Image) - Camera feed at 30Hz
- `/f450/camera/camera_info` (sensor_msgs/CameraInfo) - Camera parameters

### Aggregated State Data (For Haptic Suit Integration)
- `/f450/state/pose` (geometry_msgs/PoseStamped) - Position and orientation
- `/f450/state/velocity` (geometry_msgs/TwistStamped) - Linear and angular velocities  
- `/f450/state/altitude` (std_msgs/Float64) - Current altitude (Z position)
- `/f450/state/attitude` (geometry_msgs/Vector3Stamped) - Roll, Pitch, Yaw angles (radians)
- `/f450/state/gps` (sensor_msgs/NavSatFix) - GPS coordinates

### Control Topics
- `/f450/cmd_vel` (geometry_msgs/Twist) - Velocity commands (subscribe to control)
- `/f450/enable_motors` (std_msgs/Bool) - Enable/disable motors
- `/f450/motor_status` (std_msgs/Bool) - Motor status

## Haptic Suit Integration

Subscribe to aggregated state topics for easy integration:

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float64

def pose_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    # Send to haptic suit...

def attitude_callback(msg):
    roll = msg.vector.x   # radians
    pitch = msg.vector.y  # radians
    yaw = msg.vector.z    # radians
    # Send to haptic suit...

def altitude_callback(msg):
    altitude = msg.data  # meters
    # Send to haptic suit...

rospy.init_node('haptic_interface')
rospy.Subscriber('/f450/state/pose', PoseStamped, pose_callback)
rospy.Subscriber('/f450/state/attitude', Vector3Stamped, attitude_callback)
rospy.Subscriber('/f450/state/altitude', Float64, altitude_callback)
rospy.spin()
```

## Monitoring

```bash
# List all drone topics
rostopic list | grep f450

# Echo specific data
rostopic echo /f450/state/altitude
rostopic echo /f450/state/attitude
rostopic echo /f450/state/pose

# View camera feed
rosrun image_view image_view image:=/f450/camera/image_raw

# Monitor with rqt
rqt
# Then: Plugins → Topics → Topic Monitor
```

## Advanced Usage

### Custom Start Position

```bash
roslaunch flight_controller f450_simulation.launch x:=5 y:=3 z:=1
```

### Headless Mode (No GUI)

```bash
roslaunch flight_controller f450_simulation.launch gui:=false
```

### Adjust Flight Characteristics

Edit `models/f450/model.sdf` in the Hector controller plugin section:

```xml
<maxForce>30</maxForce>              <!-- Max thrust -->
<velocityXYGain>5.0</velocityXYGain> <!-- XY response speed -->
<velocityZGain>5.0</velocityZGain>   <!-- Altitude response speed -->
<attitudeGain>10.0</attitudeGain>    <!-- Stability -->
<yawGain>2.0</yawGain>               <!-- Rotation speed -->
```

### Adjust GPS Realism

In `models/f450/model.sdf`:

```xml
<drift>0.0001 0.0001 0.0001</drift>          <!-- GPS drift over time (m/s) -->
<gaussianNoise>0.01 0.01 0.01</gaussianNoise> <!-- Position noise (m) -->
<referenceLatitude>37.4133</referenceLatitude>  <!-- GPS reference point -->
<referenceLongitude>-122.1381</referenceLongitude>
```

## Troubleshooting

### Issue: Drone falls immediately after spawn

**Cause:** Motors not enabled yet

**Solution:** Wait 2-3 seconds for the enable_motors node to activate, or manually:
```bash
rostopic pub /f450/enable_motors std_msgs/Bool "data: true" -r 10
```

### Issue: GPS shows all zeros

**Cause:** GPS takes a few seconds to initialize

**Solution:** Wait 5 seconds after spawn, then check again

### Issue: Keyboard control doesn't work

**Cause:** Terminal window not in focus

**Solution:** Click on the keyboard teleop terminal window before pressing keys

### Issue: Simulation is laggy

**Solutions:**
1. Run without GUI: `roslaunch flight_controller f450_simulation.launch gui:=false`
2. Close other applications
3. Reduce Gazebo real-time factor

### Issue: Import errors or node crashes

**Cause:** Race condition during startup

**Solution:** Kill all processes and restart:
```bash
killall -9 gazebo gzserver gzclient rosmaster roscore
roslaunch flight_controller f450_simulation.launch
```

## Architecture

```
┌─────────────────────┐
│   Gazebo Physics    │
│   (F450 Model)      │
└──────────┬──────────┘
           │
           ├─ Sensors: IMU, Camera, GPS, Odometry
           │
           ↓
┌─────────────────────────────────┐
│ Hector Quadrotor Controller     │
│ (Built-in Gazebo Plugin)        │
│ - Subscribes: /f450/cmd_vel     │
│ - Controls: Motor forces        │
└──────────┬──────────────────────┘
           │
           ↓
┌─────────────────────────────────┐
│  State Publisher Node           │
│  - Aggregates sensor data       │
│  - Publishes: /f450/state/*     │
└──────────┬──────────────────────┘
           │
           ↓
┌─────────────────────────────────┐
│     Haptic Suit System          │
│  (Your custom nodes here)       │
└─────────────────────────────────┘
```

## Files Structure

```
flight_controller/
├── launch/
│   ├── f450_simulation.launch    # Main launch file
│   └── keyboard_teleop.launch    # Standalone keyboard control
├── models/
│   └── f450/
│       ├── model.sdf             # F450 drone model with Hector plugins
│       ├── model.config
│       └── meshes/               # 3D meshes (from autonomous_landing_uav)
├── worlds/
│   └── f450_world.world          # Gazebo world file
├── src/
│   ├── f450_keyboard_teleop.py   # Keyboard control node
│   └── f450_state_publisher.py   # State aggregation node
├── CMakeLists.txt
├── package.xml
└── README.md (this file)
```

## Performance

- **Update Rate:** 1000Hz physics, 50Hz state publishing
- **Latency:** <10ms sensor data, <5ms control response
- **CPU Usage:** ~30-40% single core (Gazebo), ~5% nodes
- **Memory:** ~500MB Gazebo, ~50MB nodes

## Credits

- **Hector Quadrotor:** TU Darmstadt - Simulation, Systems Optimization and Robotics Group
- **F450 Meshes:** Autonomous Landing UAV project
- **Integration:** Haptic Suit project team

## Support

For issues:
1. Check this README's troubleshooting section
2. Verify Hector plugins are installed: `dpkg -l | grep hector-gazebo`
3. Check ROS/Gazebo versions match (Noetic/Gazebo 11)
4. Review launch file output for errors

## License

See main project LICENSE file.

