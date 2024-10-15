# -*- coding: utf-8 -*-
"""Asst3

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1juZDbFiP9POTFIQ8zcIy0SYitP9RTKQX
"""

# Commented out IPython magic to ensure Python compatibility.
# %pip install modern_robotics

# https://colab.research.google.com/drive/1juZDbFiP9POTFIQ8zcIy0SYitP9RTKQX

from modern_robotics import *
import numpy as np
import csv

# Puppet function with inputs as required

def Puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt, damping, stiffness, springPos, restLength):
  # Calculate the number of iterations
  N = int(t/dt) + 1
  print(N)

  # Calculate the number of joints
  n = len(Slist[0])

  # Create joint angle and toruqe matrix
  thetamat, dthetamat = np.zeros((N,n)), np.zeros((N,n))
  thetamat[0], dthetamat[0] = thetalist, dthetalist
  taulist = np.zeros(n)

  # Calculate M07
  M07 = np.asarray(Mlist[0]) @ np.asarray(Mlist[1]) @ np.asarray(Mlist[2]) @ np.asarray(Mlist[3]) @ np.asarray(Mlist[4]) @ np.asarray(Mlist[5]) @ np.asarray(Mlist[6])
  M07 = M07.tolist()

  for i in range(N-1):
    # Normalize angles
    thetamat[i] = np.arctan2(np.sin(thetamat[i]), np.cos(thetamat[i]))

    # Find Tbs
    Tbs = TransInv(FKinSpace(M07, Slist, thetamat[i]))

    # Convert the spring position to b frame
    dir = Tbs @ np.array([[springPos[0]], [springPos[1]], [springPos[2]], [1]])
    dir = np.array([dir[0], dir[1], dir[2]])

    # Calculate the spring length
    d = np.linalg.norm(dir) - restLength
    dir_hat = dir / np.linalg.norm(dir)

    # Caculate the force the spring acts on end effector
    F = -dir_hat * stiffness * (d)
    W = np.array([0,0,0,F[0], F[1], F[2]])

    # Solve for damping
    taulist = -dthetalist * damping

    # Perform forward dynamics and euler step
    ddthetalist = ForwardDynamics(thetalist, dthetalist, taulist, g, W, Mlist, Glist, Slist)
    thetalist,dthetalist = EulerStep(thetalist, dthetalist, ddthetalist, 1.0 * dt)

    # Save result for next iteration
    thetamat[i + 1], dthetamat[i + 1] = thetalist, dthetalist

  # Convert all NaN to 0 to indicate that the simulation did not produce good result
  thetamat = np.nan_to_num(thetamat)
  dthetamat = np.nan_to_num(dthetamat)

  # Return required matricies
  return thetamat, dthetamat

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
M07 = np.asarray(M01) @ np.asarray(M12) @ np.asarray(M23) @ np.asarray(M34) @ np.asarray(M45) @ np.asarray(M56) @ np.asarray(M67)
M07 = M07.tolist()

G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

# 1a
N=len(Slist[0])
thetalist = np.zeros(N)
dthetalist = np.zeros(N)
g = np.array([0,0,-9.81])
t = 5
dt = 0.001
damping = 0
stiffness = 0
springPos = np.zeros(3)
restLength = 0
thetamat, dthetamat = Puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt,damping, stiffness, springPos, restLength)
np.savetxt("1a.csv", thetamat, delimiter=",")

# 1b
N=len(Slist[0])
thetalist = np.zeros(N)
dthetalist = np.zeros(N)
g = np.array([0,0,-9.81])
t = 5
dt = 0.01
damping = 0
stiffness = 0
springPos = np.zeros(3)
restLength = 0
thetamat, dthetamat = Puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt,damping, stiffness, springPos, restLength)
np.savetxt("1b.csv", thetamat, delimiter=",")

# 2a
N=len(Slist[0])
thetalist = np.zeros(N)
dthetalist = np.zeros(N)
g = np.array([0,0,-9.81])
t = 5
dt = 0.01
damping = 0.6
stiffness = 0
springPos = np.zeros(3)
restLength = 0
thetamat, dthetamat = Puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt,damping, stiffness, springPos, restLength)
np.savetxt("2a.csv", thetamat, delimiter=",")

# 2b
N=len(Slist[0])
thetalist = np.zeros(N)
dthetalist = np.zeros(N)
g = np.array([0,0,-9.81])
t = 5
dt = 0.01
damping = -0.001
stiffness = 0
springPos = np.zeros(3)
restLength = 0
thetamat, dthetamat = Puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt,damping, stiffness, springPos, restLength)
np.savetxt("2b.csv", thetamat, delimiter=",")

# 3a
N=len(Slist[0])
thetalist = np.zeros(N)
dthetalist = np.zeros(N)
g = np.array([0,0,0])
t = 10
dt = 0.01
damping = 0
stiffness = 4
springPos = np.array([1,1,1])
restLength = 0
thetamat, dthetamat = Puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt,damping, stiffness, springPos, restLength)
np.savetxt("3a.csv", thetamat, delimiter=",")

# 3b
N=len(Slist[0])
thetalist = np.zeros(N)
dthetalist = np.zeros(N)
g = np.array([0,0,0])
t = 10
dt = 0.01
damping = 2
stiffness = 4
springPos = np.array([1,1,1])
restLength = 0
thetamat, dthetamat = Puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt,damping, stiffness, springPos, restLength)
np.savetxt("3b.csv", thetamat, delimiter=",")