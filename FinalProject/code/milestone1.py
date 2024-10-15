from modern_robotics import *


# Geometry to calculate F6
l = 0.47 / 2
w = 0.3 / 2
r = 0.0475
F = r / 4 * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)], [1, 1, 1, 1], [-1, 1, -1, 1]])
F6 = np.vstack((np.zeros(4, ), np.zeros(4, ), F, np.zeros(4, )))


def NextState(x, u, dt, max_wheel_speed, max_arm_speed):

    """
    Perform first order Euler step in order to compute new joint, wheel angles and chassis configuration.
    :param x: 12 vector (phi, x, y, J1, J2, J3, J4, J5, W1, W2, W3, W4)
    :param u: 9 vector (wheels speeds, joint speeds)
    :param dt: Time-step
    :param max_wheel_speed: rad/s
    :param max_arm_speed: m/s
    :return: 12 vector representing next state after time step
    """
    global F

    # Normalize to max wheel and joint speed
    for i in range(5):
        if u[4 + i] > max_arm_speed:
            u[4 + i] = max_arm_speed
        if u[4 + i] < -max_arm_speed:
            u[4 + i] = -max_arm_speed

    for i in range(4):
        if u[i] > max_wheel_speed:
            u[i] = max_wheel_speed
        if u[i] < -max_wheel_speed:
            u[i] = -max_wheel_speed


    # Compute new wheel and joint angles
    new_joint_angles = x[3:8] + u[4:9] * dt
    new_wheel_angles = x[8:12] + u[0:4] * dt


    # Perform odometry
    vb = F @ u[0:4].T
    vb = np.array([0, 0, *vb, 0])
    Tsb = np.array([[np.cos(x[0]), -np.sin(x[0]), 0, x[1]], [np.sin(x[0]), np.cos(x[0]), 0, x[2]], [0, 0, 1, 0.0963], [0, 0, 0, 1]])
    T = MatrixExp6(VecTose3(vb) * dt)
    q = Tsb @ T
    [R, p] = TransToRp(q)
    phiVec = so3ToVec(R)

    return np.array([np.arcsin(phiVec[2]), q[0][3], q[1][3], *new_joint_angles, *new_wheel_angles])


"""
Run milestone1.py to test correctness
Moves 0.475 m in x direction
"""

if __name__ == "__main__":
    x = np.zeros(12)
    u = np.array([10,10,10,10, # wheel velocity
                  0,0,0,0,0]) # joint velocity

    output = np.zeros((100, 13))
    for i in range(100):
        x = NextState(x, u,0.01, 10,10)
        output[i] = np.append(x, 1)

    np.savetxt("milestone1.csv", output, delimiter=",")