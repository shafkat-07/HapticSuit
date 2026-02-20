import numpy as np
import matplotlib.pyplot as plt
from forward_kinematics import forward_kinematics

# Create the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Constants
ky = 2
km = 3

# Motor parameters
w       = np.array([1,3,3,3])
theta   = np.array([0, 0, 0, 0])

# Convert thetha to rad
theta = np.radians(theta)

# Run the forward kinematics
Fx, Fy, Fz, Tx, Ty, Tz, Fx1, Fx2, Fx3, Fx4, Fy1, Fy2, Fy3, Fy4, Fz1, Fz2, Fz3, Fz4 = forward_kinematics(ky, km, w, theta)

# Plot lateral forces
main        = np.array([[0, 0, 0, Fx, Fy, Fz]])
original    = np.array([[0, -1, 0, Fx1, Fy1, Fz1], [0, 1, 0, Fx2, Fy2, Fz2], [1, 0, 0, Fx3, Fy3, Fz3], [-1, 0, 0, Fx4, Fy4, Fz4]])
X1, Y1, Z1, U1, V1, W1 = zip(*main)
X2, Y2, Z2, U2, V2, W2 = zip(*original)
ax.quiver(X1, Y1, Z1, U1, V1, W1, color='C0', linewidths=3)
ax.quiver(X2, Y2, Z2, U2, V2, W2, color='C1', linewidths=3)

# Plot torque forces
plotter = np.linspace(0, 2*np.pi, 201)
Tx_plot = [0*plotter, Tx*np.cos(plotter), Tx*np.sin(plotter)]
Ty_plot = [Ty*np.cos(plotter), 0*plotter, Ty*np.sin(plotter)]
Tz_plot = [Tz*np.sin(plotter), Tz*np.cos(plotter), 0*plotter]
ax.plot(Tx_plot[0], Tx_plot[1], Tx_plot[2], color='C3' if Tx < 0 else 'C2')
ax.plot(Ty_plot[0], Ty_plot[1], Ty_plot[2], color='C3' if Ty < 0 else 'C2')
ax.plot(Tz_plot[0], Tz_plot[1], Tz_plot[2], color='C3' if Tz < 0 else 'C2')

# Set plot properties
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([-10, 10])
ax.set_xlabel(r'$x$')
ax.set_ylabel(r'$y$')
ax.set_zlabel(r'$z$')

# Display plot
plt.show()
