from modern_robotics import *
import numpy as np

R13 = np.array([[-0.7071, 0, -0.7071], [0, 1, 0],[0.7071, 0, -0.7071]])
Rs2 = np.array([[-0.6964, 0.1736, 0.6964],[-0.1228, -0.9848, 0.1228], [0.7071, 0, 0.7071]])
R25 = np.array([[-0.7566, -0.1198, -0.6428],[-0.1564, 0.9877, 0],[0.6348, 0.1005, -0.7661]])
R12 = np.array([[0.7071, 0, -0.7071],[0, 1, 0],[0.7071, 0, 0.7071]])
R34 = np.array([[0.6428, 0, -0.7660],[0, 1, 0],[0.7660, 0, 0.6428]])
Rs6 = np.array([[0.9418, 0.3249, -0.0859],[0.3249, -0.9456, -0.0151],[-0.0861, -0.0136, -0.9962]])
R6b = np.array([[-1, 0, 0],[0, 0, 1],[0, 1, 0]])

Rs1 = Rs2 @ RotInv(R12)
R23 = RotInv(R12) @ R13
R45 = RotInv(R34) @ RotInv(R23) @ R25
R56 = RotInv(R25) @ RotInv(Rs2) @ Rs6
print(AxisAng3(so3ToVec(MatrixLog3(R34)))[0])
print(round(AxisAng3(so3ToVec(MatrixLog3(Rs1)))[1],3))
print(round(AxisAng3(so3ToVec(MatrixLog3(R12)))[1],3))
print(round(AxisAng3(so3ToVec(MatrixLog3(R23)))[1],3))
print(round(AxisAng3(so3ToVec(MatrixLog3(R34)))[1],3))
print(round(AxisAng3(so3ToVec(MatrixLog3(R45)))[1],3))
print(round(AxisAng3(so3ToVec(MatrixLog3(R56)))[1],3))
print(round(AxisAng3(so3ToVec(MatrixLog3(R6b)))[1],3))

Rsb = Rs6 @ R6b
print(Rsb)