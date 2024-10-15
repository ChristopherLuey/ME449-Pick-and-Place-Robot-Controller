# CONFIGURATION FILE FOR newTask
# Run main.py with _type = "newTask"

from modern_robotics import *

# Initial EE Position
Tsei = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])

# Initial and Final Cube Configuration
qi = [2, 1, 1]
qf = [0, 1.5, -3]

print("Initial Cube Config:")
print(qi)
print("Final Cube Config:")
print(qf)

Tsci = np.array([[np.cos(qi[2]), -np.sin(qi[2]), 0, qi[0]], [np.sin(qi[2]), np.cos(qi[2]), 0, qi[1]], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tscf = np.array([[np.cos(qf[2]), -np.sin(qf[2]), 0, qf[0]], [np.sin(qf[2]), np.cos(qf[2]), 0, qf[1]], [0, 0, 1, 0.025], [0, 0, 0, 1]])

# Align axis for cube gripping and standoff
R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

# Angle of gripper at pickup
theta = np.pi / 4 - np.pi/16

R2 = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

# Standoff and gripping configurations relative to cube
Tces = RpToTrans(R @ R2, np.array([0.01, 0, 0.2]))
Tceg = RpToTrans(R @ R2, np.array([0.01, 0, -0.005]))

# Robot Geometry
Blist = np.array([[0, 0, 1, 0, 0.033, 0], [0, -1, 0, -0.5076, 0, 0], [0, -1, 0, -0.3526, 0, 0], [0, -1, 0, -0.2176, 0, 0], [0, 0, 1, 0, 0, 0]]).T
Mlist = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])
Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])

# Initial Robot Configuration
x = np.array([-np.pi/4, -0.5, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

Kp = np.zeros((6, 6))
Ki = np.zeros((6, 6))

# Gains
_Kp = 2
_Ki = 0.05

for i in range(6):
    Kp[i][i], Ki[i][i] = _Kp, _Ki


# Max joint and wheel velocities
jmax = 99
umax = 99


# Time control
dt = 0.01
k = 1

# IO config
np.set_printoptions(suppress=True)
np.set_printoptions(precision=3)