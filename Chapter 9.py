from modern_robotics import *

np.set_printoptions(suppress=True)

Xstart = np.array([[1,0,0,0],
                   [0,1,0,0],
                   [0,0,1,0],
                   [0,0,0,1]])

Xend = np.array([[0,0,1,1],
                   [1,0,0,2],
                   [0,1,0,3],
                   [0,0,0,1]])
print(ScrewTrajectory(Xstart, Xend, Tf=10, N=10, method=3))
print(CartesianTrajectory(Xstart, Xend, 10, 10, 5))

print(QuinticTimeScaling(Tf=5, t=3))

ForwardDynamics()
ForwardDynamicsTrajectory()