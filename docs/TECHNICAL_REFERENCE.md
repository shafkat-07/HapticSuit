# HapticSuit Technical Reference

This document answers technical questions about the haptic bridge math, downwash force model, latency measurement, and UDP protocol. All formulas and behavior are taken from the codebase.

---

## 1. Haptic Bridge Math: Force → Motor Commands

**Question:** How does `haptic_bridge.py` map a 3D force vector \((F_x, F_y, F_z)\) to motor commands \((w_1, w_2, w_3, w_4)\)? Is there a transformation matrix or mapping algorithm?

**Answer:** There is **no single transformation matrix**. The mapping is done in three stages.

### 1.1 Force from IMU

Acceleration is read from `/mavros/imu/data` (Gazebo physics → ArduPilot → MAVROS). Force is computed as \(\mathbf{F} = m\,\mathbf{a}\) with gravity removed from the z-axis:

- \(F_x = m\,a_x\)
- \(F_y = m\,a_y\)
- \(F_z = m\,(a_z - 9.81)\)

`DRONE_MASS` is 1.5 kg (Iris-class quad).

### 1.2 Deadzone

Small forces are zeroed to ignore simulation jitter:

- If \(|F_x|,\, |F_y|,\, |F_z| < 0.2\) → that component is set to 0.

### 1.3 Inverse Kinematics (IK)

The mapping from target force (and zero target torque) to motor speeds and angles is **numerical optimization**, not a closed-form matrix.

- **Solver:** `KinematicsSolver` in `drone_ws/src/haptic_interface/scripts/kinematics_solver.py`, using **scipy.optimize.minimize** with method **SLSQP** (Sequential Least SQuares Programming).
- **Unknowns:** \([w_1, w_2, w_3, w_4, q_1, q_2, q_3, q_4]\) with bounds \(w_i \in [0, 100]\), \(q_i \in [-\pi, \pi]\).
- **Forward kinematics (used inside the solver):**
  - Per-motor force magnitude: \(f_i = k_y\,w_i\) (default \(k_y = 2.0\)).
  - Per-motor torque magnitude: \(t_i = \pm k_m\,w_i\) (default \(k_m = 3.0\); motors 2 and 4 use the negative sign).
  - Motor 1 & 2 contribute to X and Z: \(f_{x,i} = -f_i\sin(q_i)\), \(f_{z,i} = f_i\cos(q_i)\), \(f_{y,i} = 0\).
  - Motor 3 & 4 contribute to Y and Z: \(f_{y,i} = -f_i\sin(q_i)\), \(f_{z,i} = f_i\cos(q_i)\), \(f_{x,i} = 0\).
  - Total force and torque are the sum of these contributions (see `kinematics_solver.py` for the full torque expressions).
- **Objective:** Minimize weighted squared error between this forward output and target \([F_x, F_y, F_z, 0, 0, 0]\), with weights `[1.0, 1.0, 1.0, 0.1, 0.1, 0.1]` (forces weighted more than torques), plus a small regularization term on the \(w_i\) to limit motor effort.

So the “mapping algorithm” is **inverse kinematics via nonlinear optimization**; the same structure is reflected in the MATLAB files under `kinematics/matlab/` (e.g. `kinematic_forward.m`, `kinematic_inverse.m`).

---

## 2. Downwash Equations (`downwash_force.py`)

**Question:** What equations are used for “simplified momentum theory” and the “Dryden-like turbulence overlay”?

### 2.1 Simplified Momentum Theory

- **Induced velocity at the rotor disk:**
  \[
  v_{\mathrm{induced}} = \sqrt{\frac{T}{2\,\rho\,A}},\qquad T = m\,g,\quad A = \pi R^2.
  \]
  With \(R = 0.25\,\mathrm{m}\), \(\rho = 1.225\,\mathrm{kg/m}^3\), \(T = 1.5 \times 9.81\,\mathrm{N}\).

- **Downwash velocity** at the victim drone (Gaussian radial decay + vertical decay):
  \[
  \sigma = \sigma_{\mathrm{base}} + \sigma_{\mathrm{rate}}\cdot \Delta z,
  \]
  \[
  \text{radial\_decay} = \exp\left(-\frac{r_{\mathrm{horiz}}^2}{2\sigma^2}\right),\quad
  \text{vertical\_decay} = \left(\frac{R}{\max(\Delta z,\, R)}\right)^2,
  \]
  \[
  v_{\mathrm{wash}} = 2\,v_{\mathrm{induced}} \cdot \text{vertical\_decay} \cdot \text{radial\_decay}.
  \]

- **Base downward drag force** (bluff-body drag):
  \[
  F_{\mathrm{base}} = \frac{1}{2}\,\rho\,v_{\mathrm{wash}}^2\,C_D\,A_{\mathrm{eff}}.
  \]
  With \(C_D = 1.0\), \(A_{\mathrm{eff}} = 0.04\,\mathrm{m}^2\). This is applied as a downward force; turbulence and a small radial term are added (see below).

### 2.2 Dryden-Like Turbulence Overlay

- **Single-axis model:** First-order low-pass filtered white noise (implemented per axis in `DrydenNoise`):
  \[
  \alpha = 1 - \exp(-2\pi\cdot \text{bandwidth}\cdot \Delta t),\qquad
  x_{n+1} = x_n + \alpha\,(w_n - x_n),\quad w_n \sim \mathcal{N}(0,1).
  \]
  With `TURB_BANDWIDTH = 5.0` Hz and \(\Delta t = 1/50\) s.

- **Force overlay:**
  \[
  F_z = -\bigl(F_{\mathrm{base}} + F_{\mathrm{base}}\cdot \tau\cdot n_z\bigr),\quad
  F_x = F_{\mathrm{base}}\cdot \tau\cdot \lambda\cdot n_x,\quad
  F_y = F_{\mathrm{base}}\cdot \tau\cdot \lambda\cdot n_y,
  \]
  with \(\tau = \texttt{TURB\_INTENSITY} = 0.40\), \(\lambda = \texttt{TURB\_LATERAL\_SCALE} = 0.25\). An optional radial outward term \(0.15\,F_{\mathrm{base}}\,(r_{\mathrm{horiz}}/\sigma)\) is added in the horizontal direction.

**For papers:** You can cite **actuator-disk momentum theory** for \(v_{\mathrm{induced}}\) and \(v_{\mathrm{wash}}\), **quadratic drag** for \(F_{\mathrm{base}}\), and a **first-order linear filter on Gaussian white noise** for the Dryden-inspired turbulence (not the full Dryden spectrum).

---

## 3. Latency Test Design

**Question:** How is latency measured—ROS timestamps from wrench application in Gazebo to UDP packet generation, or physical hardware response time?

**Answer:** The repository does **not** implement a dedicated latency test. There is no code that:

- Measures time from a wrench applied in Gazebo (or from a known ROS timestamp) to the moment the corresponding UDP packet is sent, or  
- Measures physical hardware response time (suit motion vs. command).

The validation guide and other docs describe applying a wrench and checking that the bridge and mock suit react, but not timestamp-based latency measurement. To report latency in a paper, you would need to add such instrumentation (e.g. ROS timestamps at wrench application and at UDP send, and optionally hardware timing).

---

## 4. UDP Protocol

**Question:** What is the structure of the UDP packet sent to the suit (e.g. JSON, byte array)?

**Answer:** The packet is **plain text CSV**, not JSON and not a binary struct.

- **Format:** One line per packet:  
  `q1,q2,q3,q4,w1,w2,w3,w4`
- **Encoding:** `packet.encode()` (UTF-8) then sent with `sock.sendto(..., (SUIT_IP, SUIT_PORT))` (default port 8888).
- **Semantics:**
  - `q1`–`q4`: motor angles in **degrees** (one decimal place).
  - `w1`–`w4`: motor speeds/throttles in the range 0–100 (one decimal place).

**Example:** `"45.0,-45.0,0.0,0.0,80.5,80.5,0.0,0.0"`

The receiver (e.g. `mock_suit.py`) expects exactly 8 comma-separated values and parses them as floats. So the UDP payload is a **short CSV string in UTF-8**, not JSON and not a fixed binary layout.

---

## References in Codebase

| Topic              | Primary file(s) |
|--------------------|-----------------|
| Bridge + IK        | `drone_ws/src/haptic_interface/scripts/haptic_bridge.py`, `kinematics_solver.py` |
| Downwash + Dryden  | `drone_ws/src/flight_controller/src/downwash_force.py` |
| UDP format / mock  | `haptic_bridge.py` (packet format), `mock_suit.py` (receiver) |
| Kinematics derivation | `kinematics/` (MATLAB, plotter, README) |
