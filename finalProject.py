from modern_robotics import *
import numpy as np


def FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt):
	Adj_X_err = MatrixLog6(TransInv(X) @ Xd)
	X_err_R = Adj_X_err[0:3][0:3]
	X_err_p_so3 = Adj_X_err[3:6][0:3] @ RotInv(X_err_R)
	X_err_p = so3ToVec(X_err_p_so3)
	X_err = RpToTrans(X_err_R, X_err_p)


	Vd = 1/dt * MatrixLog6(TransInv(Xd) @Xdnext)
	V = Adjoint(TransInv(X))@Adjoint(Xd) @ Vd + Kp@X_err + Ki@X_err * dt

	return V

def NextState(x, u, dt, max_wheel_speed, max_arm_speed):

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

	vb = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1,1,1,1], [-1,1,-1,1]]) @ x[8:12].T
	phi = x[0]
	_x = x[1]
	_y = x[2]
	Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, _x], [np.sin(phi), np.cos(phi), 0, _y],[0,0,1,0.0963],[0,0,0,1]])
	T = MatrixExp6(vb)
	q = Tsb@T

	return new_joint_angles, new_wheel_angles, q


# Helper function to convert trajectory into proper output form
def ConvertTrajectory(traj, output, n, gripper):
	for i in traj:
		rotation, position = TransToRp(i)
		new_arr = np.append(np.append(np.reshape(rotation, 9), position),gripper)
		output[n] = new_arr
		n+=1
	return n


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


# Example of using trajectory generation
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

output = TrajectoryGeneration(Tsei, Tsci, Tscf, Tceg, Tces, 1)
np.savetxt("output.csv", output, delimiter=",")
