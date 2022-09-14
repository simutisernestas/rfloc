import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.optimize import least_squares
from filterpy.kalman import KalmanFilter


class Beacon:

    def __init__(self, x0=np.zeros((3, 1))) -> None:
        if x0.shape != (3, 1):
            raise Exception("Wrong state shape!")
        self.__x = x0

    def get_pos(self) -> np.array:
        return self.__x


class Agent:

    def __init__(self, x0=np.zeros((9, 1))) -> None:
        self.__x = x0

    def update(self, state) -> None:
        if state.shape != (9, 1):
            raise Exception("Wrong state shape!")
        self.__x = state

    def get_state(self) -> np.array:
        return self.__x

    def triangulate_pos(self, beacons: list):
        A = np.zeros((3, 3))

        def dist(a, b):
            return np.linalg.norm(a - b, 2)
        ds = np.array(  # distances to beacons (ground truth)
            # [dist(agent.get_state()[:3], b.get_pos())for b in beacons])
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

    def advance(self, F) -> None:
        self.__x = F @ self.__x


def mapp(beacons: list, ax0: np.array, path: np.array, gt_path: np.array):
    legends = []
    legends.append("Path")
    legends.append("GT")
    for i, b in enumerate(beacons):
        x = b.get_pos()
        plt.scatter(x[0], x[1])
        legends.append(str(i))
    plt.scatter(ax0[0], ax0[1], marker='x', s=200)
    legends.append("Agent")
    plt.plot(path[:, 0], path[:, 1])
    plt.plot(gt_path[:, 0], gt_path[:, 1])
    plt.legend(legends)
    plt.show()


def update(x, P, Z, H, R):
    y = Z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.pinv(S)
    Xprime = x + K @ y
    KH = K @ H
    Pprime = (np.eye(KH.shape[0]) - KH) @ P
    return (Xprime, Pprime)


def predict(x, P, F, u):
    Xprime = F @ x + u
    Pprime = F @ P @ F.T
    return (Xprime, Pprime)


if __name__ == '__main__':
    def gen_beacon_on_the_groud():
        # point
        r = np.random.random((3, 1))
        r -= .5
        r *= 100
        r[2][0] = np.random.random() * 3
        return Beacon(x0=r)
    beacons = [gen_beacon_on_the_groud() for _ in range(4)]

    ax0 = np.ones((9, 1))
    ax0[:3] = (np.random.random((3, 1)) - .5) * 50
    if ax0[2] < 0:  # above ground
        ax0[2] *= -1
    agent = Agent(x0=ax0.copy())

    f = KalmanFilter(dim_x=9, dim_z=3)
    f.x = agent.get_state()

    # Time step
    dt = 1e-2

    # The initial uncertainty (9x9).
    f.P = np.eye(9, 9) * 1000

    # The external motion (9x1).
    u = np.zeros((9, 1))

    # The transition matrix (9x9).
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
    f.F = F

    # The observation matrix (3x9).
    f.H = np.zeros((3, 9))
    f.H[0][0] = 1
    f.H[1][1] = 1
    f.H[2][2] = 1

    # The measurement uncertainty.
    f.R *= 300

    path = []
    gt_path = []
    counter = 0
    while True:
        # ground truth based on physics
        agent.advance(F)
        gt_path.append(agent.get_state()[:2])
        if counter > 1000:
            break
        counter += 1
        f.predict()
        Z = agent.triangulate_pos(beacons)
        f.update(Z)
        path.append(f.x)

    # print(Z, agent.get_state()[:3])
    print(f"||X - GT|| = {np.linalg.norm(f.x[:3] - agent.get_state()[:3])}")
    print(agent.get_state(), '\n\n', f.x)

    path = np.array(path)
    gt_path = np.array(gt_path)
    mapp(beacons, ax0, path, gt_path)
