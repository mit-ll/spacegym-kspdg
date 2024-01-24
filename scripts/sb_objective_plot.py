# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# a script for visualizing the sunblocking objective function

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def sb_objective(r, t, r_d=100, A=1, B=1e-5):
    return - A * np.exp(-B * (r-r_d)**2) * np.cos(t)

if __name__ == "__main__":

    # setup range and angle arrays
    r = np.linspace(0, 1000, 100)
    t = np.linspace(0, 2*np.pi, 100)
    R, T = np.meshgrid(r, t)

    # evaluate function on meshgrid
    Z = sb_objective(R, T)

    # Create a 3D plot of the function
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(R*np.cos(T), R*np.sin(T), Z, cmap='viridis')
    ax.set_xlabel('x-pos [m]')
    ax.set_ylabel('y-po [m]')
    ax.set_zlabel('reward [-]')
    plt.show()