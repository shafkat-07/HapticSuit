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



---

## Scenario: Wind Only (Fly Straight)

**World file**: `wind_only.world`
**Launch file**: `../launch/wind_only_simulation.launch`

### Description

An Iris drone takes off, flies 5 m in a straight line, and holds position. During flight the drone experiences:
- **Constant wind** at 5 m/s in the +X direction (applied via Gazebo's native `WindPlugin`).

There is no suspended payload in this scenario. It uses the standard `iris_with_ardupilot` model from the upstream `ardupilot_gazebo` package. This makes it useful as a baseline for isolating wind-only effects on the haptic suit output, without the additional pendulum dynamics of the payload scenario.

The entire scenario is fully automated: arming, takeoff, and flight all happen via the python script.

### How to Run

You need **2 terminals**. Source ROS and the workspace in each one first:

```bash
source /opt/ros/noetic/setup.bash
source ~/source/HapticSuit/drone_ws/devel/setup.bash
```

**Terminal 1 -- Gazebo + MAVROS + Haptic Bridge:**

```bash
roslaunch flight_controller wind_only_simulation.launch
```

**Terminal 2 -- Flight Script:**

```bash
rosrun flight_controller fly_straight_wind.py
```

### What You Should See

1.  **Gazebo window**: The Iris drone takes off to 2m, waits 10s, then flies 5m forward in the +X direction. The wind (5m/s) is active, though the visual effect on the drone body may be subtle with the standard model.
2.  **Terminal 2**: The script logs progress: connecting to FCU, switching to GUIDED mode, arming, taking off, and flying to the target.
3.  **Haptic suit**: Receives UDP packets reflecting the forces the drone is experiencing (mainly wind drag).

### Tuning Parameters

| Parameter | File | Default | Description |
|---|---|---|---|
| Wind speed/direction | `wind_only.world` | `5 0 0` | Change `<linear_velocity>` |
| Flight Distance | `fly_straight_wind.py` | `5.0` m | `target_pose.pose.position.x` |
| Hover Altitude | `fly_straight_wind.py` | `2.0` m | `target_pose.pose.position.z` |

---

## Scenario: Wind + Suspended Payload

**World file**: `wind_payload.world`
**Launch file**: `../launch/wind_payload_simulation.launch`

### Description

Similar to the Wind Only scenario, but with a **0.5 kg spherical payload** suspended 0.3m below the drone via a ball joint. The drone flies the same 5m straight path in 5 m/s wind.

This scenario adds **pendulum dynamics** to the haptic feedback. As the drone accelerates or fights the wind, the payload swings, creating varying forces on the drone frame which are transmitted to the haptic suit.

### How to Run

**Terminal 1 -- Gazebo + MAVROS + Haptic Bridge:**

```bash
roslaunch flight_controller wind_payload_simulation.launch
```

**Terminal 2 -- Flight Script:**

```bash
rosrun flight_controller fly_straight_wind.py
```

### What You Should See

1.  **Gazebo window**: The drone takes off with a red spherical payload hanging below it. The payload will swing as the drone maneuvers and reacts to the wind.
2.  **Terminal 2**: Same flight sequence as the wind-only scenario.
3.  **Haptic suit**: Forces will now include the oscillating pull of the swinging payload in addition to the wind drag.

---

## Scenario: Collision + Suspended Payload

**World file**: `collision_payload.world`
**Launch file**: `../launch/collision_payload_simulation.launch`

### Description

In this scenario, there is **no wind**. The drone (with suspended payload) flies a straight path that is obstructed by a stationary object.

- **Obstacle**: A red 1x1x1m box located at `x=2.5, y=0, z=1.3`. The top of the box is at `z=1.8m`.
- **Flight Path**: From origin `(0,0,2)` to target `(5,0,2)`.
- **Payload**: Suspended approx 0.3m below the drone (at `z~1.7m`).

The drone (at `z=2.0m`) should **fly over** the box, but the **hanging payload** should collide with it. This tests the haptic response to payload impact forces while the drone remains relatively stable (initially).

### How to Run

**Terminal 1 -- Gazebo + MAVROS + Haptic Bridge:**

```bash
roslaunch flight_controller collision_payload_simulation.launch
```

**Terminal 2 -- Flight Script:**

```bash
rosrun flight_controller fly_straight_collision.py
```

### What You Should See

1.  **Gazebo window**: Drone takes off to 2m, flies forward. The drone body clears the red box, but the spherical payload dragging below smashes into it.
2.  **Haptic suit**: Should receive a sharp tug/impact force as the payload snags on the obstacle.
