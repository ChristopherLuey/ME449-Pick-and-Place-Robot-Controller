from milestone1 import *
from milestone2 import *
from milestone3 import *
from util import *

"""
HOW TO USE
2. Run best.py to generate the best configuration
3. A successful log is as follows:

py best.py
Running best configuration
Generating EE trajectory
Generating configuration output
Done.

"""

_type = "best"
enhancements = False


if _type == "best":
	from best_config import *
elif _type == "overshoot":
	from overshoot_config import *
elif _type == "newTask":
	from newTask_config import *
else:
	print("Type not defined")
	exit()



# Saving files to proper directory
parent_directory = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
directory_path = os.path.join(parent_directory, 'results', _type)

if not os.path.exists(directory_path):
	os.makedirs(directory_path)

if enhancements:
	_type = _type + "_enhancements"

f = open(os.path.join(directory_path, '{}_log.txt'.format(_type)), "w")
print("py best.py".format(_type), file=f)
print("Running {} configuration".format(_type), file=f)


# GENERATE EE TRAJECTORY
print("Generating EE trajectory", file=f)
traj = TrajectoryGeneration(Tsei, Tsci, Tscf, Tceg, Tces, k)
#np.savetxt("trajectory.csv", traj, delimiter=",")

# Uncomment next line if another trajectory is to be used
# traj = np.genfromtxt("trajectory.csv", delimiter=",")


N = traj.shape[0]-1
output = np.zeros((int(round(N/k)),13))
X_err = np.zeros((int(round(N/k)), 6))
X_err_int = np.zeros(6)

print("Generating configuration output",file=f)


# Main loop for kinematics
for i in range(N):

	T0e = FKinBody(Mlist, Blist, x[3:8])
	Tsb = np.array([[np.cos(x[0]), -np.sin(x[0]), 0, x[1]], [np.sin(x[0]), np.cos(x[0]), 0, x[2]], [0,0,1,0.0963], [0,0,0,1]])


	# Determine current and desired EE positions
	X = Tsb@Tb0@T0e
	Xd = TransformationFromList(traj[i])
	Xdnext = TransformationFromList(traj[i+1])

	V, err = FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt/k, X_err_int)
	X_err_int += err * dt / k

	# Solve Jacobians for base and arm
	Jbase = Adjoint(TransInv(T0e) @ TransInv(Tb0)) @ F6
	Jarm = JacobianBody(Blist, x[3:8])
	J = np.hstack((Jbase, Jarm))

	if enhancements:
		Jt = np.linalg.pinv(J, rcond=10.0**(-2))
	else:
		Jt = np.linalg.pinv(J)

	# Calculate new wheel and joint velocities
	x_dot = Jt @ V

	x = NextState(x, x_dot, dt/k, umax, jmax)
	x = np.append(x, traj[i][12])

	# Save only if multiple of k
	if i % k == 0:
		output[int(i/k)] = x
		X_err[int(i/k)] = err

os.path.join(directory_path, '{}_out.csv'.format(_type))
np.savetxt(os.path.join(directory_path, '{}_out.csv'.format(_type)), output, delimiter=",")
np.savetxt(os.path.join(directory_path, '{}_error.csv'.format(_type)), X_err, delimiter=",")
ErrorPlot(X_err, _type, directory_path)
print("Done.",file=f)
f.close()
