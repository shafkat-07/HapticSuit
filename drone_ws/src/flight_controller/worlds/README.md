# Simulation Scenarios

Each scenario is a self-contained Gazebo world file paired with a launch file. The drone experiences forces from the physics engine, and the haptic suit mirrors those forces in real-time via `haptic_bridge.py`.

## Prerequisites

All scenarios require the following to be installed and configured before running.

### 1. ROS Noetic

Standard ROS 1 install on Ubuntu 20.04. See [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation).

### 2. ArduPilot SITL

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

### 3. ardupilot_gazebo (Iris Model + Gazebo Plugin)

```bash
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 4. Environment Variables

Add to `~/.bashrc`:

```bash
# ArduPilot Gazebo
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH}
```

Then `source ~/.bashrc`.

### 5. Build the Workspace

```bash
cd ~/source/HapticSuit/drone_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

---

## Scenario: Wind + Suspended Payload

**World file**: `wind_payload.world`
**Launch file**: `../launch/wind_payload_simulation.launch`

### Description

An Iris drone hovers while experiencing:
- **Constant wind** at 5 m/s in the +X direction (applied via Gazebo's native `WindPlugin`).
- **A 0.5 kg suspended payload** attached to the drone's base via a ball joint (tether). The payload swings under wind and drone movement.

The combined forces (wind drag + payload pendulum dynamics) are felt by the drone through the physics engine. The drone's IMU reports these forces, MAVROS publishes them, and `haptic_bridge.py` translates them into haptic suit motor commands in real-time.

### How to Run

You need **3 terminals**. Source ROS and the workspace in each one first:

```bash
source /opt/ros/noetic/setup.bash
source ~/source/HapticSuit/drone_ws/devel/setup.bash
```

**Terminal 1 -- Gazebo + MAVROS + Haptic Bridge:**

```bash
roslaunch flight_controller wind_payload_simulation.launch
```

**Terminal 2 -- ArduPilot SITL:**

```bash
cd ~/ardupilot/ArduCopter
python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

Wait until MAVProxy shows `APM: EKF3 IMU0 is using GPS` or similar EKF ready messages.

**Terminal 3 -- Arm and Fly (in MAVProxy console or a separate terminal):**

In the MAVProxy console from Terminal 2:

```
mode GUIDED
arm throttle
takeoff 2
```

### What You Should See

1. **Gazebo window**: The Iris drone takes off. A small red sphere (payload) hangs below, swinging due to wind and movement. The drone should visibly drift or tilt slightly in the +X direction from the wind.

2. **MAVProxy console**: Shows the drone is armed, in GUIDED mode, and flying. Altitude should settle near 2m.

3. **ROS topics** (check with `rostopic echo`):
   - `/mavros/imu/data` -- IMU accelerations will show non-zero X forces from wind/payload.
   - `/haptic_bridge/debug` -- Shows `[Fx, Fy, Fz, w1, w2, w3, w4]` being sent to the suit.

4. **Haptic suit** (or `mock_suit.py`): Receives UDP packets on `127.0.0.1:8888` with motor angles and speeds reflecting the forces the drone is experiencing. To monitor without hardware:

```bash
rosrun haptic_interface mock_suit.py
```

### Tuning Parameters

| Parameter | File | Default | Description |
|---|---|---|---|
| Wind speed/direction | `wind_payload.world` line 7 | `5 0 0` (5 m/s +X) | Change `<linear_velocity>X Y Z</linear_velocity>` |
| Wind force scaling | `wind_payload.world` line 46 | `0.5` | `<force_approximation_scaling_factor>` |
| Payload mass | `../models/suspended_payload/model.sdf` line 7 | `0.5` kg | `<mass>` value |
| Payload size | `../models/suspended_payload/model.sdf` lines 19,25 | `0.05` m radius | `<radius>` in collision and visual |
| Drone mass (haptic calc) | `haptic_bridge.py` line 21 | `1.5` kg | `DRONE_MASS` constant |
