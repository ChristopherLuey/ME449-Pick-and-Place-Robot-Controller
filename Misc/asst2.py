from modern_robotics import *
import csv
import matplotlib.pyplot as plt

np.set_printoptions(suppress=True)


# Iterative function
def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
	thetalist = np.array(thetalist0).copy()

	# Lists to output
	_thetalist= []
	_pos = []
	_ev = []
	_eomg = []

	# Iterations
	i = 0
	maxiterations = 500

	# Calcualte initial error and twist
	Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, thetalist)), T)))
	err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev


	while err and i < maxiterations:
		thetalist = thetalist + np.dot(np.linalg.pinv(JacobianBody(Blist, thetalist)), Vb)

		# Normalize theta between -2pi, 2pi
		for j in range(len(thetalist)):
			if not (-2* np.pi < thetalist[j] < 2* np.pi):
				thetalist[j] = thetalist[j] % (2*np.pi)

		i+=1

		# Calculate transformation, twist and error
		_T = FKinBody(M, Blist, thetalist)
		Vb = se3ToVec(MatrixLog6(np.dot(TransInv(_T), T)))
		eomg_ = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
		ev_ = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
		err = eomg_ > eomg or ev_ > ev

		# Print results
		print()
		print("Iteration {}".format(i))
		print("joint vector: {}".format(thetalist))
		print("SE(3) end-effector config:\n{}".format(_T))
		print("error twist V_b: {}".format(Vb))
		print("angular error ||omega_b||: {}".format(eomg_))
		print("linear error ||v_b||: {}".format(ev_))

		# Save information to lists for plotting
		_thetalist.append(thetalist.tolist())
		_pos.append([i, _T[0: 3, 3].tolist()])
		_eomg.append([i, eomg_])
		_ev.append([i, ev_])


	# Determine if short or long csv
	if i <= 5: out = "short_iterates.csv"
	else: out = "long_iterates.csv"

	# Print to csv
	f=open(out, "w", newline='')
	csvWriter = csv.writer(f)
	csvWriter.writerows(_thetalist)

	return (thetalist, not err, _pos, _eomg, _ev)

# Initial guesses
thetalist_long = np.array([-1.4, 3.2,1.5, 5.2, 3.0,3.3])
thetalist_short = np.array([1.6, 3.2,1.9, 5.2, 3.4,3.3])

# Initial conditions
w1=0.109
w2=0.082
l1=0.425
l2=0.392
h1=0.089
h2=0.095

M = np.array([[-1,0,0,l1+l2],
              [0,0,1,w1+w2],
              [0,1,0,h1-h2],
              [0,0,0,1]])

Blist = np.array([[0,1,0,w1+w2,0,l1+l2],
                  [0,0,1,h2,-l1-l2,0],
                  [0,0,1,h2,-l2,0],
                  [0,0,1,h2,0,0],
                  [0,-1,0,-w2,0,0],
                  [0,0,1,0,0,0]]).T

T = np.array([[0.7071,0,0.7071,-0.3],[0.7071,0,-0.7071,-0.5],[0,1,0,0.5],[0,0,0,1]])

# Running long and short iterations
long = IKinBodyIterates(Blist,M,T, thetalist_long, 0.001,0.0001)
short = IKinBodyIterates(Blist,M,T, thetalist_short, 0.001,0.0001)



# Plotting for question 4
x1, y1, z1 = zip(*[point[1] for point in long[2]])
x2, y2, z2 = zip(*[point[1] for point in short[2]])

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x1, y1, z1, color='blue', marker='o', label='Long (32 iterations)')
ax.scatter(x2, y2, z2, color='red', marker='o', label='Short (4 iterations)')
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')

# Draw lines between points
for i in range(1, len(long[2])):
	ax.plot([x1[i-1], x1[i]], [y1[i-1], y1[i]], [z1[i-1], z1[i]], color='blue')

for i in range(1, len(short[2])):
    ax.plot([x2[i-1], x2[i]], [y2[i-1], y2[i]], [z2[i-1], z2[i]], color='red')

# Denote the starting and ending points with markers or labels
ax.scatter(x1[0], y1[0], z1[0], color='green', marker='o', s=100, label='Start')
ax.scatter(x2[0], y2[0], z2[0], color='green', marker='o', s=100, label='Start')
ax.scatter(x1[-1], y1[-1], z1[-1], color='purple', marker='x', s=100, label='End')
ax.legend()


# Plot for questions 5 and 6
iterate_number1, error_magnitude1 = zip(*long[3])
iterate_number2, error_magnitude2 = zip(*short[3])
plt.figure(2, figsize=(8, 6))
plt.plot(iterate_number1, error_magnitude1, marker='o', linestyle='-', label='Long (32 Iterations)')
plt.plot(iterate_number2, error_magnitude2, marker='x', linestyle='--', label='Short (4 Iterations)')
plt.title('Magnitude of Angular Error vs. Iterate Number')
plt.xlabel('Iterate Number')
plt.ylabel('Magnitude of Angular Error')
plt.legend()
plt.grid(True)

iterate_number1, error_magnitude1 = zip(*long[4])
iterate_number2, error_magnitude2 = zip(*short[4])
plt.figure(3, figsize=(8, 6))
plt.plot(iterate_number1, error_magnitude1, marker='o', linestyle='-', label='Long (32 Iterations)')
plt.plot(iterate_number2, error_magnitude2, marker='x', linestyle='--', label='Short (4 Iterations)')
plt.title('Magnitude of Linear Error vs. Iterate Number')
plt.xlabel('Iterate Number')
plt.ylabel('Magnitude of Linear Error')
plt.legend()
plt.grid(True)

# Display the plot
plt.show()
