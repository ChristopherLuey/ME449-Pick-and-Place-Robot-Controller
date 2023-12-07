from modern_robotics import *
import numpy as np

np.set_printoptions(suppress=True)
np.set_printoptions(precision=3)


def FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt):
	Vd = se3ToVec(1/dt * MatrixLog6(TransInv(Xd) @Xdnext))
	adxinvxdvd = Adjoint(TransInv(X)@Xd) @ Vd
	X_err = se3ToVec(MatrixLog6(TransInv(X) @ Xd))
	V = adxinvxdvd + Kp@X_err + Ki@X_err * dt
	return V


def NextState(x, u, dt, max_wheel_speed, max_arm_speed):

	# A 12-vector representing the current configuration of the robot (3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles).
	# A 9-vector of controls indicating the wheel speeds � {\displaystyle u} (4 variables) and the arm joint speeds  � ˙{\displaystyle {\dot {\theta }}} (5 variables).

	for i in range(5):
		if u[4+i]> max_arm_speed:
			u[4+i] = max_arm_speed
		if u[4+i] < -max_arm_speed:
			u[4+i] = -max_arm_speed

	for i in range(4):
		if u[i] > max_wheel_speed:
			u[i] = max_wheel_speed
		if u[i] < -max_wheel_speed:
			u[i] = -max_wheel_speed

	new_joint_angles = x[3:8] + u[4:9] * dt
	new_wheel_angles = x[8:12] + u[0:4] * dt

	l = 0.47/2
	w=0.3/2
	r=0.0475

	vb = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1,1,1,1], [-1,1,-1,1]]) @ u[0:4].T
	vb = np.array([0,0,*vb,0])
	phi = x[0]
	_x = x[1]
	_y = x[2]
	Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, _x], [np.sin(phi), np.cos(phi), 0, _y],[0,0,1,0.0963],[0,0,0,1]])
	T = MatrixExp6(VecTose3(vb)*dt)
	q = Tsb@T
	[R, p] = TransToRp(q)
	phi = so3ToVec(R)

	return np.array([phi[2], q[0][3], q[1][3], *new_joint_angles, *new_wheel_angles])


# Helper function to convert trajectory into proper output form
def ConvertTrajectory(traj, output, n, gripper):
	for i in traj:
		rotation, position = TransToRp(i)
		new_arr = np.append(np.append(np.reshape(rotation, 9), position),gripper)
		output[n] = new_arr
		n+=1
	return n


def TransformationFromList(row):
	return np.array([[row[0], row[1], row[2], row[9]], [row[3], row[4], row[5], row[10]],[row[6], row[7], row[8], row[11]],[0,0,0,1]])


# TrajectoryGeneration for Milestone 2
def TrajectoryGeneration(Tsei, Tsci, Tscf, Tceg, Tces, k):

	# Set time for each trajectory and calculate number of steps a function of k
	# Could do this intelligently by considering distances and might implement later
	time = [5, 2, 1, 2, 5, 2, 1, 2]
	N = [int(i * k/0.01) for i in time]
	output = np.zeros((sum(N), 13))

	# n to track
	n = 0

	# List to track trajectory frames
	traj = [Tsci @ Tces, Tsci @ Tceg, Tsci @ Tceg, Tsci @ Tces, Tscf @ Tces, Tscf @ Tceg, Tscf @ Tceg, Tscf @ Tces]
	curr, gripper = Tsei, 0
	for index, value in enumerate(traj):
		t = CartesianTrajectory(curr, value, time[index], N[index], 5)
		if index == 2 or index == 6:
			gripper = 1-gripper # Reverse state of gripper when grabbing and releasing cube
		n = ConvertTrajectory(t, output, n, gripper)
		curr = value # Track previous reference frame

	return output


Tsei = np.array([[0,0,1,0], [0,1,0,0], [-1,0,0,0.5], [0,0,0,1]])
Tsci = np.array([[1,0,0,1], [0,1,0,0], [0,0,1,0.025], [0,0,0,1]])
Tscf = np.array([[0,1,0,0], [-1,0,0,-1], [0,0,1,0.025], [0,0,0,1]])

# Align axis for cube gripping and standoff
R=np.array([[0,0,1], [0,1,0], [-1,0,0]])
# Control the angle of gripper
theta = np.pi/4
R2 = np.array([[np.cos(theta), 0, np.sin(theta)],[0, 1, 0],[-np.sin(theta), 0, np.cos(theta)]])

Tces = RpToTrans(R@R2, np.array([0,0,0.13]))
Tceg = RpToTrans(R@R2, np.array([-0.008, 0, 0.0004]))



Blist = np.array([[0,0,1,0,0.033,0],[0,-1,0,-0.5076,0,0], [0,-1,0,-0.3526,0,0], [0,-1,0,-0.2176,0,0], [0,0,1,0,0,0]]).T
Mlist = np.array([[1,0,0,0.033], [0,1,0,0], [0,0,1,0.6546], [0,0,0,1]])
Tb0 = np.array([[1,0,0,0.1662], [0,1,0,0], [0,0,1,0.0026], [0,0,0,1]])

x = np.array([0,0,0,0,0,0.2,-1.6,0,0,0,0,0,0])

Kp = np.zeros((6,6))


Kp = np.zeros((6,6))
Ki = np.zeros((6,6))
_Kp = 1
_Ki = 0.1

for i in range(6):
	Kp[i][i] = _Kp
	Ki[i][i] = _Ki

l = 0.47 / 2
w = 0.3 / 2
r = 0.0475

dt = 0.01
k = 1

traj = TrajectoryGeneration(Tsei, Tsci, Tscf, Tceg, Tces, k)
np.savetxt("trajectory.csv", traj, delimiter=",")

N = traj.shape[0]-1
output = np.zeros((int(N/k),13))


for i in range(N):

	T0e = FKinBody(Mlist, Blist, x[3:8])
	Tsb = np.array([[np.cos(x[0]), -np.sin(x[0]), 0, x[1]], [np.sin(x[0]), np.cos(x[0]), 0, x[2]], [0,0,1,0.0963], [0,0,0,1]])
	X = Tsb@Tb0@T0e

	Xd = TransformationFromList(traj[i])
	Xdnext = TransformationFromList(traj[i+1])

	V = FeedbackControl(X, Xd, Xdnext,Kp, Ki, dt)

	F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1,1,1,1], [-1,1,-1,1]])
	F6 = np.vstack((np.zeros(4,), np.zeros(4,), F, np.zeros(4,)))

	Jbase = Adjoint(TransInv(T0e) @ TransInv(Tb0)) @ F6
	Jarm = JacobianBody(Blist, x[3:8])

	J = np.hstack((Jbase, Jarm))
	Jt = J.T @ np.linalg.inv(J @ J.T)

	x_dot = Jt @ V

	x = NextState(x, x_dot, 0.01, 10, 10)
	x = np.append(x, traj[i][12])

	if i%k == 0:
		output[i] = x


np.savetxt("out.csv", output, delimiter=",")
