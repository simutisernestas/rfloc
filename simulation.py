import numpy as np
import matplotlib.pyplot as plt
# from scipy.optimize import minimize
# from scipy.optimize import least_squares
# from filterpy.kalman import KalmanFilter
# from filterpy.common import Q_discrete_white_noise
# from filterpy.kalman import ExtendedKalmanFilter
# from typing import Lists
# from typing import Callable


MYEFK = True

class Beacon:

    def __init__(self, x0=np.zeros((3, 1))) -> None:
        if x0.shape != (3, 1):
            raise Exception("Wrong state shape!")
        self.__x = x0

    def get_pos(self) -> np.array:
        return self.__x


def dist(a, b):
    return np.linalg.norm(a - b, 2)


class Agent:

    def __init__(self, x0=np.zeros((9, 1))) -> None:
        self.__x = x0

    def update(self, state) -> None:
        if state.shape != (9, 1):
            raise Exception("Wrong state shape!")
        self.__x = state

    def get_state(self) -> np.array:
        return self.__x

    def get_beacon_dists(self) -> np.array:
        return np.array(
            [dist(agent.get_state()[:3], b.get_pos()) + np.random.random() for b in beacons])

    def triangulate_pos(self): # , beacons: List[Beacon]
        A = np.zeros((3, 3))

        ds = np.array(  # distances to beacons
            # [dist(agent.get_state()[:3], b.get_pos())for b in beacons]) <= (ground truth)
            [dist(agent.get_state()[:3], b.get_pos()) + np.random.random() for b in beacons])

        bp = [b.get_pos() for b in beacons]

        A[0][0] = (bp[1] - bp[0])[0]
        A[1][0] = (bp[2] - bp[0])[0]
        A[2][0] = (bp[3] - bp[0])[0]

        A[0][1] = (bp[1] - bp[0])[1]
        A[1][1] = (bp[2] - bp[0])[1]
        A[2][1] = (bp[3] - bp[0])[1]

        A[0][2] = (bp[1] - bp[0])[2]
        A[1][2] = (bp[2] - bp[0])[2]
        A[2][2] = (bp[3] - bp[0])[2]

        b = np.zeros((3, 1))

        k1 = np.sum(bp[0]*bp[0])
        k2 = np.sum(bp[1]*bp[1])
        k3 = np.sum(bp[2]*bp[2])
        k4 = np.sum(bp[3]*bp[3])

        b[0] = (ds[0]**2 - ds[1]**2 - k1 + k2)
        b[1] = (ds[0]**2 - ds[2]**2 - k1 + k3)
        b[2] = (ds[0]**2 - ds[3]**2 - k1 + k4)

        return np.linalg.inv(2*A) @ b

    def advance(self) -> None:
        # linear motion model
        F = np.eye(9)
        F[0][3] = dt
        F[1][4] = dt
        F[2][5] = dt
        F[0][6] = .5*dt**2
        F[1][7] = .5*dt**2
        F[2][8] = .5*dt**2
        F[3][6] = dt
        F[4][7] = dt
        F[5][8] = dt
        self.__x = F @ self.__x


def mapp(beacons: list, ax0: np.ndarray, path: np.ndarray, gt_path: np.ndarray):
    plt.figure(dpi=300)
    legends = []
    for i, b in enumerate(beacons):
        x = b.get_pos()
        plt.scatter(x[0], x[1])
        legends.append(str(i))
    plt.scatter(ax0[0], ax0[1], marker='x', s=200)
    legends.append("x0")
    plt.plot(path[:, 0], path[:, 1])
    plt.plot(gt_path[:, 0], gt_path[:, 1])
    legends.append("Path")
    legends.append("GT")
    plt.legend(legends)
    plt.show()


def update(x, hx, P, Z, H, R):
    y = Z - hx
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    Xprime = x + K @ y
    KH = K @ H
    I_KH = (np.eye(KH.shape[0]) - KH)
    # Pprime = I_KH @ P @ I_KH.T + K @ R @ K.T
    Pprime = I_KH @ P
    return (Xprime, Pprime)


def predict(x, P, F, u):
    Xprime = F @ x + u
    Pprime = F @ P @ F.T + np.eye(6) # TODO: handle process noise
    return (Xprime, Pprime)


def dist_jac(x_op: np.ndarray, beacons): # , beacons: List[Beacon]
    """    
    pr - robot position
    pbi = beacon_i position
    J = [
      d/dx h(x) = norm(pr - pb1), 
      d/dx h(x) = norm(pr - pb2),
      ... 
      d/dx h(x) = norm(pr - pbn),
    ]
    first row of J (distance function h(x) to the beacon 1):
    -(bx - x)/((bx - x)^2 + (by - y)^2 + (bz - z)^2)^(1/2)
    -(by - y)/((bx - x)^2 + (by - y)^2 + (bz - z)^2)^(1/2)
    -(bz - z)/((bx - x)^2 + (by - y)^2 + (bz - z)^2)^(1/2)
    or switch bx,x places and remove minus in front

    Parameters
    ----------
    x_op : np.ndarray (3,1)
        operating point i.e. agent's previous position.
    """
    H = np.zeros((len(beacons), len(x_op)))
    for i, b in enumerate(beacons):
        H[i][:3] = ((x_op[:3] - b.get_pos()) / np.linalg.norm(x_op[:3] - b.get_pos())).T
    return H

if __name__ == '__main__':
    def gen_beacon_on_the_groud():
        # point
        r = np.random.random((3, 1))
        r -= .5
        r *= 100
        r[2][0] = np.random.random() * 3
        return Beacon(x0=r)
    beacons = [gen_beacon_on_the_groud() for _ in range(4)]

    ax0 = np.zeros((9, 1))
    ax0[:3] = (np.random.random((3, 1)) - .5) * 50
    if ax0[2] < 0:  # above ground
        ax0[2] *= -1
    ax0[3] = 10  # go right at 10 m/s
    agent = Agent(x0=ax0.copy())

    dt = 1e-2

    path = []
    gt_path = []

    if MYEFK:
        dim_x = 6
        dim_z = 4
        x = agent.get_state()[:6] + np.random.random((6, 1))*3
        F = np.eye(dim_x)
        F[0,3] = dt
        F[1,4] = dt
        F[2,5] = dt
        range_std = 1  # meters
        R = np.diag([range_std**2]*4)
        P = np.eye(dim_x)
        P *= 50
    else:
        rk = ExtendedKalmanFilter(dim_x=6, dim_z=4)
        rk.x = agent.get_state()[:6] + np.random.random((6, 1))*3
        rk.F = np.eye(6)
        rk.F[0,3] = dt
        rk.F[1,4] = dt
        rk.F[2,5] = dt
        range_std = .5  # meters
        rk.R = np.diag([range_std**2]*4)
        rk.P *= 10

    def HJacobian_at(x, beacons):
        return dist_jac(x[:6], beacons)

    def hx(x, beacons):
        h = np.zeros((4, 1))
        for i, b in enumerate(beacons):
            h[i] = np.linalg.norm(x[:3] - b.get_pos())
        return h

    for i in range(4):
        counter = 0
        while True:
            # ground truth based on physics
            agent.advance()
            gt_path.append(agent.get_state()[:2])
            if counter > 200:
                break
            counter += 1
            z = agent.get_beacon_dists()

            if MYEFK:
                (x,P) = update(x, hx(x, beacons), P, np.array([z]).T, HJacobian_at(x, beacons), R)
                path.append(x)
                (x,P) = predict(x, P, F, 0)
            else:
                rk.update(np.array([z]).T, HJacobian_at, hx,
                   args=(beacons), hx_args=(beacons))
                path.append(rk.x)
                rk.predict()
 
        state = agent.get_state().copy()
        state[3:6] = 0
        if i == 0:  # go up at 10 m/s
            state[4] = 10
        elif i == 1:  # go left at 12 m/s
            state[3] = -12
        elif i == 2:  # go down at 10 m/s
            state[4] = -10
        agent.update(state)


    if MYEFK:
        print(f"Var: {np.max(P)}\n"
              f"Dist: {np.linalg.norm(x[:3] - agent.get_state()[:3])}\n")
        print(agent.get_state(), '\n\n', x)
        print(P)
    else:
        print(f"Var: {np.max(rk.P)}\n"
            #   f"Dist: {np.linalg.norm(x[:3] - agent.get_state()[:3])}\n")
        f"Dist: {np.linalg.norm(rk.x[:3] - agent.get_state()[:3])}\n")
        print(agent.get_state(), '\n\n', rk.x)
        print(rk.P)

    path = np.array(path)
    gt_path = np.array(gt_path)
    mapp(beacons, ax0, path, gt_path)



# r**3 variance dependand on distances
# v cov
# make the scale of axes in the plot the same, so covariance is seen
# maybe plot the covariance matrix
# and think about 3d plotting
# write some python code to extract the data from tag, through serial connection
# ask about time line of the project ? can i do the report after new years
# how does gain traslate distance to pos ??