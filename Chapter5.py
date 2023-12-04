from modern_robotics import *
import numpy as np
np.set_printoptions(precision=2)
np.set_printoptions(suppress=True)


J = np.array([
	[0,0,0],
	[0,0,0],
	[1,1,1],
	[0,0, 1/np.sqrt(2)],
	[0,-1,-2/np.sqrt(2)],
	[0,0,0]
])

F = np.array([2,0,0])
print(F)
print(J.T)
print(np.matmul(J,F))


theta1 = 0
theta2 = 0
theta3 = np.pi/2
theta4 = -np.pi/2
l1=1
l2=1
l3=1
l4=1

Jb = np.array([[0,0,0,0],[0,0,0,0],[1,1,1,1],
               [l3*np.sin(theta4) + l2 * np.sin(theta3 + theta4) + l2 * np.sin(theta2+theta3+theta4), l3*np.sin(theta4) + l2*np.sin(theta3+theta4), l3*np.sin(theta4),0],
               [l4+l3*np.cos(theta4) + l2*np.cos(theta3+theta4) + l1*np.cos(theta2 + theta3+ theta4), l4 + l3*np.cos(theta4) + l2 * np.cos(theta3+ theta4), l4 + l3*np.cos(theta4),l4],
               [0,0,0,0]])
Fb = np.array([0,0,10,10,10,0])

print(Jb.T@Fb)

[[0,0,1,0,0,0], [1,0,0,0,2,0], [0,0,0,0,1,0]]
p = JacobianSpace([[0,1,0],[0,0,0],[1,0,0],[0,0,0],[0,2,1],[0,0,0]], [np.pi/2, np.pi/2, 1])
print(repr(p))
p = JacobianBody([[0,-1,0],[1,0,0],[0,0,0],[3,0,0],[0,3,0],[0,0,1]], [np.pi/2, np.pi/2, 1])
print(repr(p))





V=np.array([[-0.105, 0, 0.006, -0.045,0,0.006,0],[-0.889,0.006,0,-0.844,0.006,0,0],[0,-0.105, 0.889, 0,0,0,0]])
A = V@V.T
y = np.linalg.eig(A)
print(A)
print(y)
print(np.sqrt(y[0][1]))
