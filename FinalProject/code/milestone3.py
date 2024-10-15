from modern_robotics import *

# Converts a transformation from list for to ndarray
def TransformationFromList(row):
    return np.array([[row[0], row[1], row[2], row[9]], [row[3], row[4], row[5], row[10]], [row[6], row[7], row[8], row[11]], [0, 0, 0, 1]])


def FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt, X_err_int):
    """
    Uses kinematic task-space feedforward and feedback to determine the instructed end-effector motion based on the intended and current end-effector

    X: Current EE position (se(3))
    Xd: Current desired EE position (se(3))
    Xdnext: Next desired EE position (se(3))
    Kp: Proportional matrix (6x6)
    Ki: Integral matrix (6x6)
    dt: Time step
    X_err_int: Accumulating error vector (6)

    :return: Twist control and EE error
    """
    Vd = se3ToVec(1 / dt * MatrixLog6(TransInv(Xd) @ Xdnext))
    adxinvxdvd = Adjoint(TransInv(X) @ Xd) @ Vd
    X_err = se3ToVec(MatrixLog6(TransInv(X) @ Xd))
    V = adxinvxdvd + Kp @ X_err + Ki @ (X_err_int + X_err * dt)
    return V, X_err


# Run milestone3.py to test
if __name__ == "__main__":
    Xd = np.array([[0,0,1,0.5], [0,1,0,0], [-1,0,0,0.5], [0,0,0,1]])
    Xdnext = np.array([[0,0,1,0.6], [0,1,0,0], [-1,0,0,0.3], [0,0,0,1]])
    X = np.array([[0.170, 0, 0.985, 0.387], [0,1,0,0],[-0.985, 0, 0.170, 0.570], [0,0,0,1]])
    Kp = np.zeros((6,6))
    Ki = np.zeros((6,6))
    dt = 0.01
    X_err_int = np.zeros(6)
    V, err = FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt, X_err_int)
    print(V, err)
