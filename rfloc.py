import numpy as np
import matplotlib.pyplot as plt


class Beacon:

    def __init__(self, id, x0=np.zeros((3, 1))) -> None:
        if x0.shape == (3,) or x0.shape == (1, 3):
            x0 = x0.reshape((3, 1))
        if x0.shape != (3, 1):
            raise Exception("Wrong state shape!")
        self.__x = x0
        self.__id = id
        self.__range = None
        self.__last_update = 0
        self.__LIFESPAN = .1  # s

    def get_pos(self) -> np.array:
        return self.__x

    def update_range(self, distance, stamp):
        self.__range = distance
        self.__last_update = stamp

    def get_range(self, stamp):
        if abs(stamp - self.__last_update) > self.__LIFESPAN:
            self.__range = None
        return self.__range

    def is_active(self):
        return self.get_range() is not None

    def get_id(self):
        return self.__id


class Agent:

    def __init__(self, x0=np.zeros((9, 1))) -> None:
        self.__x = x0
        self.__last_update = 0

    def update(self, state, stamp) -> None:
        if state.shape != (6, 1):
            raise Exception("Wrong state shape!")
        self.__x = state
        self.__last_update = stamp

    def get_state(self) -> np.array:
        return self.__x

    def get_last_update(self):
        return self.__last_update

    def get_beacon_dists(self, beacons) -> np.array:
        return np.array(
            [np.linalg.norm(self.get_state()[:3] - b.get_pos()) + np.random.random() for b in beacons])

    def triangulate_pos(self, beacons) -> np.array:
        A = np.zeros((3, 3))

        ds = np.array(  # distances to beacons
            # [np.linalg.norm(self.get_state()[:3], b.get_pos())for b in beacons]) <= (ground truth)
            [np.linalg.norm(self.get_state()[:3] - b.get_pos()) + np.random.random() for b in beacons])

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

    def advance(self, dt) -> None:
        # linear motion model
        F = getF(dt)
        self.__x = F @ self.__x


def mapp(beacons: list, ax0: np.ndarray, path: np.ndarray, gt_path: np.ndarray):
    plt.figure(dpi=150)
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
    Pprime = F @ P @ F.T + np.eye(6)  # TODO: handle process noise
    return (Xprime, Pprime)


def getH(x_op: np.ndarray, beacons):  # , beacons: List[Beacon]
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
        H[i][:3] = ((x_op[:3] - b.get_pos()) /
                    np.linalg.norm(x_op[:3] - b.get_pos())).T
    return H


def hx(x, beacons):
    """
    non-linear measurement func
    """
    h = np.zeros((4, 1))
    for i, b in enumerate(beacons):
        h[i] = np.linalg.norm(x[:3] - b.get_pos())
    return h


def getF(dt):
    F = np.eye(6)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    return F
