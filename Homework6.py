from modern_robotics import *
import numpy as np
import sympy

x=[1,1]
f=np.array([[x[0]**2 -9, x[1]**2 - 4]])
J=np.array([[2*x[0]-9, 2*x[1]-4]])
print(J.shape)
for i in range(2):
	Jt = np.linalg.inv(J.T @ J) @ J.T
	temp = Jt * e
	x = [x[0] + temp[0], x[1] + temp[1]]
	print(x)
	f = np.array([[x[0] ** 2 - 9, x[1] ** 2 - 4]])
	J = np.array([2 * x[0] - 9, 2 * x[1] - 4])

print(x)