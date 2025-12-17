# Haptic Suit Simulation Verification Guide

This guide outlines the steps to run the simulation, validate the data pipeline, and troubleshoot physics interactions.

## 1. Start the System

Open three separate terminals:

**Terminal 1: Mock Suit Receiver**
This script mimics the physical haptic suit, receiving and displaying motor commands via UDP.
```bash
python3 drone_ws/src/haptic_interface/scripts/mock_suit.py
```

**Terminal 2: Main Simulation Launch**
This starts Gazebo, ArduPilot SITL, MAVROS, and the Haptic Bridge.
```bash
roslaunch haptic_interface start_simulation.launch
```
*Wait for Gazebo to load and the drone to appear on the runway.*

**Terminal 3: Validation & Control**
Use this terminal for issuing ROS commands to test the physics.

---

## 2. Takeoff Procedure

You need to command the drone to fly using the **MAVProxy Console** (the white xterm window titled `sim_vehicle.py` or `Console` that appeared).

Type the following commands into that xterm window:
```bash
mode guided
arm throttle
takeoff 10
```
*The drone should climb to 10 meters.*

---

## 3. Test Physics & Haptics

Since the simulated wind parameter (`SIM_WIND_SPD`) sometimes acts as sensor noise rather than physical force, we will use ROS to apply a direct physical force (wrench) to the drone body.

**In Terminal 3 (Validation), run:**

```bash
rosservice call /gazebo/apply_body_wrench "{body_name: 'iris::base_link', reference_frame: 'world', wrench: { force: { x: 50.0, y: 0.0, z: 0.0 }, torque: { x: 0.0, y: 0.0, z: 0.0 } }, start_time: { sec: 0, nsec: 0 }, duration: { sec: 2, nsec: 0 }}"
```

### Expected Results:
1.  **Visual:** The drone in Gazebo should tilt or drift significantly for 2 seconds.
2.  **Terminal 2 (Launch):** You should see logs like:  
    `[INFO] TURBULENCE! F: [48.2, 0.1, -9.8] -> Motors: ...`
3.  **Terminal 1 (Mock Suit):** The data stream should change from zeros to active motor commands:
    ```
    [10:45:01.234] Received: 45.0,-45.0,0.0,0.0,80.5,80.5,0.0,0.0
    ```

---

## 4. Troubleshooting

*   **Mock Suit shows only zeros:**
    *   The bridge filters out small forces (< 0.2). Ensure you apply a large enough force (e.g., 50N).
    *   Verify MAVROS is connected (Terminal 2 should show `CON: Got HEARTBEAT`).
*   **Drone doesn't move with `apply_body_wrench`:**
    *   Ensure the `body_name` is correct (`iris::base_link`). You can check available bodies by running `rostopic list` or looking at the Gazebo model tree.
*   **xterm didn't open:**
    *   Ensure `xterm` is installed: `sudo apt-get install xterm`
    *   Check `sitl_launcher.py` logs in Terminal 2.

