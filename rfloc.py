import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


class Beacon:

    def __init__(self, id: str, x0: np.ndarray, lifespan=.2) -> None:
        if x0.shape == (3,) or x0.shape == (1, 3):
            x0 = x0.reshape((3, 1))
        if x0.shape != (3, 1):
            raise Exception("Wrong state shape!")
        self.__x = x0
        self.__id = id
        self.__range = None
        self.__last_update = 0
        self.__LIFESPAN = lifespan  # s

    def get_pos(self) -> np.array:
        return self.__x

    def update_range(self, distance, stamp):
        self.__range = distance - self.__get_bias(distance)
        self.__last_update = stamp

    def __get_bias(self, d):
        # Linear fit coeffs from data
        # [0.03678752 0.26399485]
        b = 0.26399485
        a = 0.03678752
        return (a*d + b)

    def get_range(self, stamp):
        if abs(stamp - self.__last_update) > self.__LIFESPAN:
            self.__range = None
        return self.__range

    def is_active(self):
        return self.get_range() is not None

    def get_id(self):
        return self.__id

    def discard_meas(self):
        self.__range = None


class Agent:

    def __init__(self, x0=np.zeros((6, 1))) -> None:
        self.__x = x0
        self.__last_update = None

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


def mapp(beacons: list, ax0: np.ndarray, path: np.ndarray, gt_path: np.ndarray = None, Ps=None, threed=False):
    plt.figure(1, dpi=150)
    plt.tight_layout()
    plt.axis('equal')
    legends = []
    plt.plot(path[:, 0].T[0], path[:, 1].T[0])
    legends.append("Path")

    for i, b in enumerate(beacons):
        x = b.get_pos()
        plt.scatter(x[0], x[1], s=100)
        legends.append("Beacon " + str(i))
    plt.scatter(ax0[0], ax0[1], marker='x', s=100)
    legends.append("x0")

    if gt_path is not None:
        plt.plot(gt_path[:, 0], gt_path[:, 1])
    legends.append("GT")
    plt.legend(legends)
    plt.savefig("report/figures/2d_path.png")

    if threed:
        plt.figure(2, dpi=150)
        plt.tight_layout()
        plt.axis('equal')
        ax = plt.axes(projection='3d')
        ax.plot3D(path[:, 0].T[0], path[:, 1].T[0], path[:, 2].T[0])
        # legends.append("Path")

        for i, b in enumerate(beacons):
            x = b.get_pos()
            ax.scatter3D(x[0], x[1], x[2], s=100)
            legends.append(str(i))
        ax.scatter3D(ax0[0], ax0[1], ax0[2], marker='x', s=100)
        # legends.append("x0")

        if gt_path is not None:
            ax.plot3D(gt_path[:, 0], gt_path[:, 1], gt_path[:, 2])
        # legends.append("GT")
        ax.legend(legends)
        plt.savefig("report/figures/3d_path.png")


def plot_gt_vs_data_with_cov(path, gt, Ps, beacons):
    plt.figure(123, dpi=150)
    legends = []

    ax = plt.subplot(111, aspect='equal')
    ax.plot(path[:, 0].T[0], path[:, 1].T[0])
    legends.append("Path")

    for i, b in enumerate(beacons):
        x = b.get_pos()
        ax.scatter(x[0], x[1], s=100)
        legends.append("Beacon " + str(i))

    if gt is not None:
        ax.plot(gt[:, 0], gt[:, 1])
    legends.append("GT")
    ax.legend(legends)

    for i, P in enumerate(Ps):
        if i % 11 != 0:
            continue
        P = P[:2, :2]
        lambda_, v = np.linalg.eig(P)
        nstd = 3
        w, h = 2 * nstd * np.sqrt(lambda_)
        ell = Ellipse(xy=(path[i, 0], path[i, 1]),
                      width=w, height=h,
                      angle=np.rad2deg(np.arccos(v[0, 0])), color='black',
                      linestyle='--')
        ell.set_facecolor('none')
        ax.add_artist(ell)
    plt.savefig("report/figures/2d_with_cov.png")

def update(x, hx, P, Z, H, R):
    y = Z - hx
    S = H @ P @ H.T + R
    try:
        K = P @ H.T @ np.linalg.inv(S)
    except:
        raise Exception("Cannot invert S matrix!")
    Xprime = x + K @ y
    KH = K @ H
    I_KH = (np.eye(KH.shape[0]) - KH)
    # Pprime = I_KH @ P @ I_KH.T + K @ R @ K.T
    Pprime = I_KH @ P
    return (Xprime, Pprime)


def predict(x, P, F, u, Pnoise=1):
    Xprime = F @ x + u
    Pprime = F @ P @ F.T + np.eye(6)*Pnoise
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
        diff = x_op[:3] - b.get_pos()
        if (diff == 0.0).all():
            raise Exception("Division by zero")
        H[i][:3] = (diff / np.linalg.norm(diff)).T
    return H


def hx(x, beacons):
    """
    non-linear measurement func
    """
    h = np.zeros((len(beacons), 1))
    for i, b in enumerate(beacons):
        h[i] = np.linalg.norm(x[:3] - b.get_pos())
        # h[i] = np.linalg.norm(x[:3] - b.get_pos())
    return h


def getF(dt):
    F = np.eye(6)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    return F
