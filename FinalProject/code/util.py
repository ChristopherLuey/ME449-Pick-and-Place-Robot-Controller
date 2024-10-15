import matplotlib.pyplot as plt
import os
import numpy as np

# Used to plot error graphs and save as a png in corresponding directory

def ErrorPlot(X_err, _type, directory_path):
    N = X_err.shape[0]
    time = np.arange(0, N * 0.01, 0.01)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

    for i, label in enumerate(['$\\theta_x$', '$\\theta_y$', '$\\theta_z$']):
        ax1.plot(time, X_err[:, i], label=label)

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angular Error (rad)')
    ax1.set_title('Angular Error (rad) vs. Time (s)')
    ax1.legend()
    #ax1.set_ylim(-0.8, 0.8)

    for i, label in enumerate(['$x$', '$y$', '$z$']):
        ax2.plot(time, X_err[:, i + 3], label=label)

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Cartesian Error (m)')
    ax2.set_title('Cartesian Error (m) vs. Time (s)')
    ax2.legend()
    #ax2.set_ylim(-0.5, 0.5)

    plt.tight_layout()
    plt.savefig(os.path.join(directory_path, '{}_X_err_plot.png'.format(_type)))
