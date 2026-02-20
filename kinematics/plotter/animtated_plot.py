import os
import copy
import numpy as np
import matplotlib.pyplot as plt

from tqdm import tqdm
from forward_kinematics import forward_kinematics

# Create the results directory
if not os.path.isdir("./results"):
    os.makedirs("./results")

speed = 100

# Create the motor parameters
w       = np.array([1.0, 1.0, 1.0, 1.0])
theta   = np.array([0.0, 0.0, 0.0, 0.0])

w_final     = []
theta_final = []


for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp1 = np.linspace(1.0, 3.0, num=int(1 * speed), endpoint=True)
t_tmp = np.linspace(0.0, 0.0, num=int(1 * speed), endpoint=True)
for x, y in zip(w_tmp1, t_tmp):
    w[0] = x
    w[1] = x
    theta[0] = y
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp1 = np.linspace(3.0, 2.0, num=int(1 * speed), endpoint=True)
w_tmp2 = np.linspace(1.0, 2.0, num=int(1 * speed), endpoint=True)
t_tmp = np.linspace(0.0, 0.0, num=int(1 * speed), endpoint=True)
for x1, x2, y in zip(w_tmp1, w_tmp2, t_tmp):
    w[0] = x1
    w[1] = x1
    w[2] = x2
    w[3] = x2
    theta[0] = y
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(2.0, 2.0, num=int(2 * speed), endpoint=True)
t_tmp1 = np.linspace(0.0, 180.0, num=int(2 * speed), endpoint=True)
t_tmp2 = np.linspace(0.0, -180.0, num=int(2 * speed), endpoint=True)
for x, y1, y2 in zip(w_tmp, t_tmp1, t_tmp2):
    w[0]        = x
    theta[0]    = y1
    theta[1]    = y2
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(2.0, 2.0, num=int(1 * speed), endpoint=True)
t_tmp1 = np.linspace(180.0, 90.0, num=int(1 * speed), endpoint=True)
t_tmp2 = np.linspace(-180.0, -270.0, num=int(1 * speed), endpoint=True)
for x, y1, y2 in zip(w_tmp, t_tmp1, t_tmp2):
    w[0]        = x
    theta[0]    = y1
    theta[1]    = y2
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(2.0, 0.0, num=int(1 * speed), endpoint=True)
t_tmp1 = np.linspace(90.0, 90.0, num=int(1 * speed), endpoint=True)
t_tmp2 = np.linspace(-270.0, -270.0, num=int(1 * speed), endpoint=True)
for x, y1, y2 in zip(w_tmp, t_tmp1, t_tmp2):
    w[0]        = x
    w[1]        = x
    theta[0]    = y1
    theta[1]    = y2
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(0.0, 2.0, num=int(1 * speed), endpoint=True)
t_tmp1 = np.linspace(90.0, 90.0, num=int(1 * speed), endpoint=True)
t_tmp2 = np.linspace(-270.0, -270.0, num=int(1 * speed), endpoint=True)
for x, y1, y2 in zip(w_tmp, t_tmp1, t_tmp2):
    w[0]        = x
    w[1]        = x
    theta[0]    = y1
    theta[1]    = y2
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(2.0, 0.0, num=int(1 * speed), endpoint=True)
t_tmp1 = np.linspace(90.0, 90.0, num=int(1 * speed), endpoint=True)
t_tmp2 = np.linspace(-270.0, -270.0, num=int(1 * speed), endpoint=True)
for x, y1, y2 in zip(w_tmp, t_tmp1, t_tmp2):
    w[2]        = x
    w[3]        = x
    theta[0]    = y1
    theta[1]    = y2
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(0.0, 2.0, num=int(1 * speed), endpoint=True)
t_tmp1 = np.linspace(90.0, 90.0, num=int(1 * speed), endpoint=True)
t_tmp2 = np.linspace(-270.0, -270.0, num=int(1 * speed), endpoint=True)
for x, y1, y2 in zip(w_tmp, t_tmp1, t_tmp2):
    w[2]        = x
    w[3]        = x
    theta[0]    = y1
    theta[1]    = y2
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(2.0, 2.0, num=int(1 * speed), endpoint=True)
t_tmp1 = np.linspace(90.0, 0.0, num=int(1 * speed), endpoint=True)
t_tmp2 = np.linspace(-270.0, -360.0, num=int(1 * speed), endpoint=True)
for x, y1, y2 in zip(w_tmp, t_tmp1, t_tmp2):
    w[0]        = x
    w[1]        = x
    theta[0]    = y1
    theta[1]    = y2
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(2.0, 2.0, num=int(2 * speed), endpoint=True)
t_tmp = np.linspace(0.0, 180.0, num=int(2 * speed), endpoint=True)
for x, y in zip(w_tmp, t_tmp):
    w[0]        = x
    theta[0]    = y
    theta[1]    = -y
    theta[2]    = y
    theta[3]    = -y
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(2.0, 2.0, num=int(1 * speed), endpoint=True)
t_tmp = np.linspace(180.0, 225.0, num=int(1 * speed), endpoint=True)
for x, y in zip(w_tmp, t_tmp):
    w[0]        = x
    theta[0]    = y
    theta[1]    = -y
    theta[2]    = y
    theta[3]    = -y
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

for _ in range(int(0.1 * speed)):
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))

w_tmp = np.linspace(2.0, 2.0, num=int(1 * speed), endpoint=True)
t_tmp = np.linspace(225.0, 135.0, num=int(1 * speed), endpoint=True)
for x, y in zip(w_tmp, t_tmp):
    w[0]        = x
    theta[0]    = y
    theta[1]    = -y
    theta[2]    = y
    theta[3]    = -y
    w_final.append(copy.copy(w))
    theta_final.append(copy.copy(theta))








# Plot the data
for i in tqdm(range(len(w_final))):

    w = w_final[i]
    theta = theta_final[i]

    # Create the figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Constants
    ky = 2
    km = 3

    # Convert thetha to rad
    theta = np.radians(theta)

    # Run the forward kinematics
    Fx, Fy, Fz, Tx, Ty, Tz, Fx1, Fx2, Fx3, Fx4, Fy1, Fy2, Fy3, Fy4, Fz1, Fz2, Fz3, Fz4 = forward_kinematics(ky, km, w, theta)

    # Plot lateral forces
    main        = np.array([[0, 0, 0, Fx, Fy, Fz]])
    original    = np.array([[0, -1, 0, Fx1, Fy1, Fz1], [0, 1, 0, Fx2, Fy2, Fz2], [1, 0, 0, Fx3, Fy3, Fz3], [-1, 0, 0, Fx4, Fy4, Fz4]])
    X1, Y1, Z1, U1, V1, W1 = zip(*main)
    X2, Y2, Z2, U2, V2, W2 = zip(*original)
    ax.quiver(X2, Y2, Z2, U2, V2, W2, color='C1', linewidths=3)
    ax.quiver(X1, Y1, Z1, U1, V1, W1, color='C0', linewidths=3)

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

    w = np.round(w, 1)
    theta = np.round(np.degrees(theta), 1)

    ax.set_title(r"$\omega$ = [{}, {}, {}, {}]".format(w[0], w[1], w[2], w[3]) + "\n" + r"$\theta$ = [{}, {}, {}, {}]".format(theta[0], theta[1], theta[2], theta[3]))

    ax.view_init(20, 45+(i/2))

    # Display plot
    plt.savefig("results/{}.png".format(str(i).zfill(4)))
    plt.close()