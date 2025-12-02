## Hardware Connection Guide (Master + Slave on Breadboard)

Since you have a breadboard, we will use the "power rails" (the long blue/red lines on the side) to make sharing Ground (GND) easy.

### 1. Set up the Common Ground (The most important step)
1.  **Master Ground:** Connect a wire from the **Master's GND** pin to the **Blue (-) Rail** on the breadboard.
2.  **Slave Ground:** Connect a wire from the **Slave's GND** pin to the **same Blue (-) Rail**.
3.  **IMU Grounds:** Connect your Master's IMU GND wire to this **same Blue (-) Rail**.
    *   *Result:* The Master, Slave, and IMUs now all share a common reference voltage.

### 2. Connect the Communication Lines (UART)
Directly connect these pins between the two Feathers (use jumper wires):
1.  **Master TX sends to Slave RX:**
    *   Connect **Master Pin 1 (TX)** $\rightarrow$ **Slave Pin 0 (RX)**.
2.  **Master RX listens to Slave TX:**
    *   Connect **Master Pin 0 (RX)** $\rightarrow$ **Slave Pin 1 (TX)**.

### 3. Verify IMU Connections (No changes, just checks)
*   **Master IMUs:** Ensure `SDA`/`SCL` are connected *only* to the **Master**.
*   **Slave IMUs:** Ensure `SDA`/`SCL` are connected *only* to the **Slave**.
*   *Note:* Do not connect the SDA/SCL lines of the Master and Slave together. They operate independently.

### Summary Checklist
- [ ] Blue Rail contains: Master GND, Slave GND, IMU GNDs.
- [ ] Master Pin 1 connected to Slave Pin 0.
- [ ] Master Pin 0 connected to Slave Pin 1.

