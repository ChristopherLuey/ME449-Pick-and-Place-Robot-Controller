from modern_robotics import *


# Helper function to convert trajectory into proper output form
def ConvertTrajectory(traj, output, n, gripper):
    for i in traj:
        rotation, position = TransToRp(i)
        new_arr = np.append(np.append(np.reshape(rotation, 9), position), gripper)
        output[n] = new_arr
        n += 1
    return n


def TrajectoryGeneration(Tsei, Tsci, Tscf, Tceg, Tces, k):
    # Set time for each trajectory and calculate number of steps a function of k
    # Could do this intelligently by considering distances and might implement later
    time = [20, 6, 2, 6, 10, 6, 2, 6]
    N = [int(i * k / 0.01) for i in time]
    output = np.zeros((sum(N), 13))

    # n to track
    n = 0

    # List to track trajectory frames
    traj = [Tsci @ Tces, Tsci @ Tceg, Tsci @ Tceg, Tsci @ Tces, Tscf @ Tces, Tscf @ Tceg, Tscf @ Tceg, Tscf @ Tces]
    curr, gripper = Tsei, 0
    for index, value in enumerate(traj):
        t = CartesianTrajectory(curr, value, time[index], N[index], 3)
        if index == 2 or index == 6:
            gripper = 1 - gripper  # Reverse state of gripper when grabbing and releasing cube
        n = ConvertTrajectory(t, output, n, gripper)
        curr = value  # Track previous reference frame

    return output


"""Test whether proper end-effector trajectory is generated"""
if __name__ == "__main__":
    k = 1
    Tsei = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])

    # Initial and Final Cube Configuration
    Tsci = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])
    Tscf = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])

    # Align axis for cube gripping and standoff
    R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

    # Angle of gripper at pickup
    theta = np.pi / 4 - np.pi / 16

    R2 = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

    # Standoff and gripping configurations relative to cube
    Tces = RpToTrans(R @ R2, np.array([0.01, 0, 0.2]))
    Tceg = RpToTrans(R @ R2, np.array([0.01, 0, -0.005]))

    traj = TrajectoryGeneration(Tsei, Tsci, Tscf, Tceg, Tces, k)
    np.savetxt("trajectory.csv", traj, delimiter=",")