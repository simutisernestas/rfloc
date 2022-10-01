import numpy as np
import matplotlib.pyplot as plt
from getdata import read_measurements

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

    def __init__(self, x0=np.zeros((6, 1))) -> None:
        self.__x = x0

    def update(self, state) -> None:
        if state.shape != (6, 1):
            raise Exception("Wrong state shape!")
        self.__x = state

    def get_state(self) -> np.array:
        return self.__x

    def get_beacon_dists(self, beacons) -> np.array:
        return np.array(
            [dist(agent.get_state()[:3], b.get_pos()) + np.random.random() for b in beacons])


def mapp(beacons: list, ax0: np.ndarray, path: np.ndarray, gt_path: np.ndarray):
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


def dist_jac(x_op: np.ndarray, beacons):  # , beacons: List[Beacon]
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


def HJacobian_at(x, beacons):
    return dist_jac(x[:6], beacons)


def hx(x, beacons):
    h = np.zeros((4, 1))
    for i, b in enumerate(beacons):
        h[i] = np.linalg.norm(x[:3] - b.get_pos())
    return h


if __name__ == '__main__':
    # initial position at 0
    ax0 = np.zeros((6, 1))
    agent = Agent(x0=ax0.copy())

    beacons = []
    while len(beacons) < 1:
        devices = read_measurements()
        # TODO: add to the beacons
        break

    exit()

    dt = 1e-1

    path = []
    gt_path = []

    # first stage would be to initialize, see enough beacons to start localization

    dim_x = 6
    dim_z = 4
    x = agent.get_state()[:6] + np.random.random((6, 1))*3
    F = np.eye(dim_x)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    range_std = 1  # meters
    R = np.diag([range_std**2]*4)
    P = np.eye(dim_x)
    P *= 50

    while True:
        z = agent.get_beacon_dists()
        (x, P) = update(x, hx(x, beacons), P, np.array(
            [z]).T, HJacobian_at(x, beacons), R)
        path.append(x)
        (x, P) = predict(x, P, F, 0)
        break

    print(f"Var: {np.max(P)}\n"
            f"Dist: {np.linalg.norm(x[:3] - agent.get_state()[:3])}\n")
    print(agent.get_state(), '\n\n', x)
    print(P)

    path = np.array(path)
    gt_path = np.array(gt_path)
    mapp(beacons, ax0, path, gt_path)
