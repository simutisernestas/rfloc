import numpy as np
import matplotlib.pyplot as plt
from getdata import read_measurements
import time

DIM_X = 6
DIM_Z = 4


class Beacon:

    def __init__(self, x0, id=None) -> None:
        if x0.shape == (3,) or x0.shape == (1, 3):
            x0 = x0.reshape((3, 1))
        if x0.shape != (3, 1):
            raise Exception("Wrong state shape!")
        self.__x = x0
        self.__id = id
        self.__last_update = 0
        self.__range = None
        self.__LIFESPAN = .1  # s

    def get_pos(self) -> np.array:
        return self.__x

    def get_id(self) -> np.array:
        return self.__id

    def update_range(self, distance):
        self.__range = distance
        self.__last_update = time.time()

    def get_range(self):
        if time.time() - self.__last_update > self.__LIFESPAN:
            self.__range = None
        return self.__range

    def is_active(self):
        return self.get_range() is not None


class Agent:

    def __init__(self, x0=np.zeros((6, 1))) -> None:
        self.__x = x0

    def update(self, state) -> None:
        if state.shape != (6, 1):
            raise Exception("Wrong state shape!")
        self.__x = state

    def get_state(self) -> np.array:
        return self.__x


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


def getF(dt):
    F = np.eye(6)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    return F


# must be one of the inputs for system
BEACON_MAP = {
    "6022": Beacon(np.array([10, 10, 0])),
    "6023": Beacon(np.array([-10, 10, 0])),
    "6024": Beacon(np.array([-10, -10, 0])),
    "6025": Beacon(np.array([10, -10, 0])),
}


def get_measurements(timeout=.1):
    start = time.time()
    found_enough_active_beacons = False
    while not found_enough_active_beacons and (time.time() - start) < timeout:
        # update from measurements
        devices = read_measurements(timeout/2)
        for id, meas in devices.items():
            BEACON_MAP[id].update_range(meas["Range"])
        # count active ones
        active_count = sum([beac.is_active()
                           for _, beac in BEACON_MAP.items()])
        if active_count >= 4:  # 4 is enough
            found_enough_active_beacons = True
    return found_enough_active_beacons, np.array([v.get_range() for k, v in BEACON_MAP.items()]).reshape((active_count, 1))


if __name__ == '__main__':
    # initial position at 0
    ax0 = np.zeros((6, 1))
    agent = Agent(x0=ax0)

    # initial ranging, find enough beacons to start localization
    success = False
    while not success:
        print("Looking for enough active beacons...")
        (success, z) = get_measurements(1)
        print(f"Found {z.shape[0]} active beacons...")
    print("Starting system...")

    path = []
    gt_path = []

    # initialize filter
    range_std = 1  # meters
    R = np.diag([range_std**2]*4)
    P = np.eye(DIM_X)
    P *= 50
    MEAS_TIMEOUT = .1

    while True:
        # TODO: populate gt_path, from opticon ?

        timestamp = time.time()
        x = agent.get_state()
        (success, z) = get_measurements(MEAS_TIMEOUT)
        if success:
            (x, P) = update(x, hx(x, BEACON_MAP.values()),
                            P, z, HJacobian_at(x, BEACON_MAP.values()), R)
        path.append(x)
        agent.update(x)
        dt = time.time() - timestamp
        (x, P) = predict(x, P, getF(dt), 0)
        break # TODO: remove

    print(f"Var: {np.max(P)}\n"
          f"Dist: {np.linalg.norm(np.zeros((3,1)) - agent.get_state()[:3])}\n")
    print(agent.get_state(), '\n\n', x)
    print(P)

    path = np.array(path)
    gt_path = np.array(gt_path)
    # mapp(beacons, ax0, path, gt_path)
