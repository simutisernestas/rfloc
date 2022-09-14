import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
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

    # TODO: x0 is very important for convergence !!!
    def triangulate_pos(self, beacons: list, debug: bool = False, x0=None):
        def error(x, c, r):
            return sum([(np.linalg.norm(x - c[i]) - r[i]) ** 2 for i in range(len(c))])

        def dist(a, b):
            return np.linalg.norm(a - b, 2)

        distances_to_station = np.array(  # TODO: increasing noise to +-0.5m brakes filter
            [dist(self.__x[:3], b.get_pos() + (np.random.random()-.5)*.1) for b in beacons])
        stations_coordinates = [b.get_pos().T for b in beacons]
        l = len(stations_coordinates)  # number of stations
        S = np.sum(distances_to_station)
        # compute weight vector for initial guess
        W = [((l - 1) * S) / (S - w) for w in distances_to_station]
        # get initial guess of point location
        if x0 is None:
            x0 = sum([W[i] * stations_coordinates[i] for i in range(l)])
        # optimize distance from signal origin to border of spheres
        sol = minimize(error, x0, args=(stations_coordinates,
                       distances_to_station), method='Nelder-Mead')
        # if not sol.success:
        #     raise Exception("Optimization failed!")
        if debug:
            print(W)
            print(x0)
            print(distances_to_station)
            print(stations_coordinates)
            print(sol)
        # if np.linalg.norm(sol.x - np.ones((3,))) > 1e-3:
        #     print(np.linalg.norm(sol.x - np.ones((3,))))
        return sol.x.reshape(3, 1), sol.success

    def advance(self, F) -> None:
        self.__x = F @ self.__x


def mapp(beacons: list, agent: Agent = None, path=None):
    legends = []
    for i, b in enumerate(beacons):
        x = b.get_pos()
        plt.scatter(x[0], x[1])
        legends.append(str(i))
    if agent is not None:
        x = agent.get_state()
        plt.scatter(x[0], x[1], marker='x')
        legends.append("Agent")
    if path is not None:
        for p in path:
            plt.scatter(p[0], p[1], color='r')
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
        r[2][0] = 0
        return Beacon(x0=r)

    beacons = [gen_beacon_on_the_groud() for _ in range(4)]
    agent = Agent(x0=np.ones((9, 1)))

    # TODO: test case for sanity
    # beacons = [
    #     Beacon(x0=[1, 1, 1]),
    #     Beacon(x0=[-1, 1, 1]),
    #     Beacon(x0=[-1, -1, -1]),
    #     Beacon(x0=[1, -1, 1])
    # ]
    # print(agent.triangulate_pos(beacons))

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
    f.R *= 1

    # x = agent.get_state()
    path = []
    counter = 0
    while True:
        # ground truth based on physics
        agent.advance(F)
        if counter > 1000:
            break
        counter += 1

        f.predict()
        (Z, success) = agent.triangulate_pos(beacons, x0=f.x[:3].T)
        if success:
            f.update(Z)
        # if np.linalg.norm(f.x[:3] - agent.get_state()[:3]) > 1:
        print(np.linalg.norm(f.x[:3] - agent.get_state()[:3]))
        print(np.linalg.norm(Z - agent.get_state()[:3].T))
        print('\n')

        # x, P = predict(x, P, F, u)
        # (Z, success) = agent.triangulate_pos(beacons)
        # if success:
        #     x, P = update(x, P, Z, H, R)
        #     # agent.update(x)
        # path.append(x[:2])
        # print(x, np.max(P), Z.T, success)
        # agent.update(x)

    print(agent.get_state(), '\n\n', f.x)
    # path = np.array(path)
    # plt.scatter(path[:, 0], path[:, 1])
    # plt.show()
    # mapp(beacons, path=path)
