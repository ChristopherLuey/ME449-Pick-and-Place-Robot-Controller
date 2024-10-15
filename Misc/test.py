from finalProject import *


x = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
u = np.array([-10,10,-10,10,0,0,0,0,0])
out = np.zeros((100, 13))

for i in range(100):
	x = np.append(NextState(x, u, 0.01, 10, 10),1)
	out[i] = x

np.savetxt("test.csv", out, delimiter=",")


