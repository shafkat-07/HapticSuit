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

An Iris drone takes off, flies 10 m in a straight line, and lands -- all fully automated. During flight the drone experiences:
- **Constant wind** at 5 m/s in the +X direction (applied via Gazebo's native `WindPlugin`).
- **A 0.5 kg suspended payload** attached to the drone's base via a ball joint (0.3 m rigid tether). The payload swings under wind and drone movement.

The drone and payload are bundled into a single composite model (`iris_with_payload`) so the tether joint lives inside the same model tree as the drone. This follows the same pattern the upstream `iris_with_ardupilot` model uses for its gimbal mount and is far more stable in Gazebo Classic than world-level joints between separate models.

The combined forces (wind drag + payload pendulum dynamics) are felt by the drone through the physics engine. The drone's IMU reports these forces, MAVROS publishes them, and `haptic_bridge.py` translates them into haptic suit motor commands in real-time.

The entire scenario is fully automated: arming, takeoff, cruise, and landing all happen without any manual input.

### How to Run

**Single terminal.** Source ROS and the workspace first:

```bash
source /opt/ros/noetic/setup.bash
source ~/source/HapticSuit/drone_ws/devel/setup.bash
```

Then launch everything:

```bash
roslaunch flight_controller wind_payload_simulation.launch
```

No additional terminals are needed. The flight script (`wind_payload_flight.py`) handles arming, mode changes, takeoff, cruise, and landing via MAVROS services automatically. An `xterm` window will open for the ArduPilot SITL console (for monitoring only -- no commands required).

### What You Should See

1. **Gazebo window**: The Iris drone takes off to 2 m, settles, then flies 10 m in the +X direction at ~1.5 m/s. A small red sphere (payload) hangs below on a thin grey tether, swinging due to wind and movement. The drone should visibly drift or tilt in the +X direction from the 5 m/s wind. After reaching the destination the drone lands autonomously.

2. **ArduPilot SITL console (xterm)**: Shows the drone is armed, in GUIDED mode, and flying. Altitude should settle near 2 m during cruise. After landing, the vehicle disarms.

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
| Wind force scaling | `wind_payload.world` line 37 | `0.5` | `<force_approximation_scaling_factor>` |
| Payload mass | `../models/iris_with_payload/model.sdf` | `0.5` kg | `<mass>` in the `payload` link |
| Payload size | `../models/iris_with_payload/model.sdf` | `0.05` m radius | `<radius>` in payload collision/visual |
| Tether length | `../models/iris_with_payload/model.sdf` | `0.3` m | Z-offset of payload geometry from link origin |
| Hover altitude | `wind_payload_flight.py` | `2.0` m | `HOVER_ALT` |
| Cruise speed | `wind_payload_flight.py` | `1.5` m/s | `CRUISE_SPEED` |
| Cruise distance | `wind_payload_flight.py` | `10.0` m | `CRUISE_DISTANCE` |
| Settle time | `wind_payload_flight.py` | `5.0` s | `SETTLE_TIME` |
| Drone mass (haptic calc) | `haptic_bridge.py` line 21 | `1.5` kg | `DRONE_MASS` constant |

---

## Scenario: Wind Only

**World file**: `wind.world`
**Launch file**: `../launch/wind_simulation.launch`

### Description

An Iris drone takes off, flies 10 m in a straight line, and lands -- all fully automated. During flight the drone experiences:
- **Constant wind** at 5 m/s in the +X direction (applied via Gazebo's native `WindPlugin`).

There is no suspended payload in this scenario. It uses the standard `iris_with_ardupilot` model from the upstream `ardupilot_gazebo` package. This makes it useful as a baseline for isolating wind-only effects on the haptic suit output, without the additional pendulum dynamics of the payload scenario.

The wind drag forces are felt by the drone through the physics engine. The drone's IMU reports these forces, MAVROS publishes them, and `haptic_bridge.py` translates them into haptic suit motor commands in real-time.

The entire scenario is fully automated: arming, takeoff, cruise, and landing all happen without any manual input.

### How to Run

**Single terminal.** Source ROS and the workspace first:

```bash
source /opt/ros/noetic/setup.bash
source ~/source/HapticSuit/drone_ws/devel/setup.bash
```

Then launch everything:

```bash
roslaunch flight_controller wind_simulation.launch
```

No additional terminals are needed. The flight script (`wind_flight.py`) handles arming, mode changes, takeoff, cruise, and landing via MAVROS services automatically. An `xterm` window will open for the ArduPilot SITL console (for monitoring only -- no commands required).

### What You Should See

1. **Gazebo window**: The Iris drone takes off to 2 m, settles, then flies 10 m in the +X direction at ~1.5 m/s. The drone should visibly drift or tilt in the +X direction from the 5 m/s wind. After reaching the destination the drone lands autonomously.

2. **ArduPilot SITL console (xterm)**: Shows the drone is armed, in GUIDED mode, and flying. Altitude should settle near 2 m during cruise. After landing, the vehicle disarms.

3. **ROS topics** (check with `rostopic echo`):
   - `/mavros/imu/data` -- IMU accelerations will show non-zero X forces from wind.
   - `/haptic_bridge/debug` -- Shows `[Fx, Fy, Fz, w1, w2, w3, w4]` being sent to the suit.

4. **Haptic suit** (or `mock_suit.py`): Receives UDP packets on `127.0.0.1:8888` with motor angles and speeds reflecting the forces the drone is experiencing. To monitor without hardware:

```bash
rosrun haptic_interface mock_suit.py
```

### Tuning Parameters

| Parameter | File | Default | Description |
|---|---|---|---|
| Wind speed/direction | `wind.world` line 7 | `5 0 0` (5 m/s +X) | Change `<linear_velocity>X Y Z</linear_velocity>` |
| Wind force scaling | `wind.world` line 31 | `0.5` | `<force_approximation_scaling_factor>` |
| Hover altitude | `wind_flight.py` | `2.0` m | `HOVER_ALT` |
| Cruise speed | `wind_flight.py` | `1.5` m/s | `CRUISE_SPEED` |
| Cruise distance | `wind_flight.py` | `10.0` m | `CRUISE_DISTANCE` |
| Settle time | `wind_flight.py` | `5.0` s | `SETTLE_TIME` |
| Drone mass (haptic calc) | `haptic_bridge.py` line 21 | `1.5` kg | `DRONE_MASS` constant |

---

## Scenario: Downwash (Drone Flyover)

**World file**: `downwash.world`
**Launch file**: `../launch/downwash_simulation.launch`

### Description

Drone A (Iris, ArduPilot SITL) hovers at 5 m AGL. A second visual-only quadcopter model (Drone B) flies over it at 7 m AGL, creating a rotor downwash disturbance on Drone A. The downwash is computed by a custom ROS node (`downwash_force.py`) using simplified momentum theory with a Dryden-like turbulence overlay, and applied to Drone A's physics body via the Gazebo `apply_body_wrench` service.

The entire scenario is fully automated: arming, takeoff, and the flyover loop all happen without any manual input. Drone B's flyover repeats in a loop so the downwash event can be observed multiple times.

**Data flow:**
Gazebo physics (+ applied downwash wrench) -> ArduPilot SITL -> MAVROS -> `haptic_bridge.py` -> Haptic Suit (UDP)

### How to Run

**Single terminal.** Source ROS and the workspace first:

```bash
source /opt/ros/noetic/setup.bash
source ~/source/HapticSuit/drone_ws/devel/setup.bash
```

Then launch everything:

```bash
roslaunch flight_controller downwash_simulation.launch
```

No additional terminals are needed. The flight script handles arming, mode changes, and takeoff via MAVROS services. An `xterm` window will open for the ArduPilot SITL console (for monitoring only -- no commands required).

### What You Should See

1. **Gazebo window**: Drone A (Iris) takes off and hovers at ~5 m. A small dark quadcopter model (Drone B) appears from one side and flies across directly over Drone A at ~7 m. When Drone B is overhead, Drone A should visibly dip or tilt from the applied downwash force. The flyover repeats every few seconds.

2. **ArduPilot SITL console (xterm)**: Shows Drone A is armed, in GUIDED mode, and maintaining altitude. You may see altitude hold corrections when the downwash pushes down.

3. **ROS topics** (check with `rostopic echo`):
   - `/mavros/imu/data` -- IMU accelerations will show disturbances during flyovers.
   - `/downwash/force` -- The raw `[Fx, Fy, Fz]` force being applied to Drone A (useful for debugging / `rqt_plot`).
   - `/haptic_bridge/debug` -- Shows `[Fx, Fy, Fz, w1, w2, w3, w4]` being sent to the suit.

4. **Haptic suit** (or `mock_suit.py`): Receives UDP packets reflecting the downwash force. Monitor without hardware:

```bash
rosrun haptic_interface mock_suit.py
```

### Tuning Parameters

| Parameter | File | Default | Description |
|---|---|---|---|
| Drone A hover altitude | `downwash_flight.py` | `5.0` m | `HOVER_ALT` |
| Drone B flyover altitude | `downwash_flight.py` | `7.0` m | `INTRUDER_ALT` |
| Flyover speed | `downwash_flight.py` | `2.0` m/s | `FLYOVER_SPEED` |
| Flyover X range | `downwash_flight.py` | `12` to `-12` m | `FLYOVER_START_X`, `FLYOVER_END_X` |
| Pause between flyovers | `downwash_flight.py` | `4.0` s | `LOOP_PAUSE` |
| Intruder mass (thrust) | `downwash_force.py` | `1.5` kg | `INTRUDER_MASS` |
| Drag coefficient | `downwash_force.py` | `1.0` | `CD` |
| Effective area | `downwash_force.py` | `0.04` m^2 | `A_EFF` |
| Turbulence intensity | `downwash_force.py` | `40%` of base | `TURB_INTENSITY` |
| Turbulence bandwidth | `downwash_force.py` | `5.0` Hz | `TURB_BANDWIDTH` |
| Drone mass (haptic calc) | `haptic_bridge.py` | `1.5` kg | `DRONE_MASS` |
