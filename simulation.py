import numpy as np
from rfloc import *


if __name__ == '__main__':
    def gen_beacon_on_the_groud():
        # point
        r = np.random.random((3, 1))
        r -= .5
        r *= 100
        r[2][0] = np.random.random() * 3
        return Beacon(int.from_bytes(np.random.bytes(2), 'little'), x0=r)
    beacons = [gen_beacon_on_the_groud() for _ in range(4)]

    ax0 = np.zeros((6, 1))
    ax0[:3] = (np.random.random((3, 1)) - .5) * 50
    if ax0[2] < 0:  # above ground
        ax0[2] *= -1
    ax0[3] = 10  # go right at 10 m/s
    agent = Agent(x0=ax0.copy())

    dt = 1e-2
    path = []
    gt_path = []
    dim_x = 6
    dim_z = 4
    x = agent.get_state()[:6] + np.random.random((6, 1))*3
    F = getF(dt)
    range_std = 1  # meters
    R = np.diag([range_std**2]*4)
    P = np.eye(dim_x)
    P *= 50

    for i in range(4):
        counter = 0
        while True:
            # ground truth based on physics
            agent.advance(dt)
            gt_path.append(agent.get_state()[:2])
            if counter > 200:
                break
            counter += 1
            z = agent.get_beacon_dists(beacons)

            (x, P) = update(x, hx(x, beacons), P, np.array(
                [z]).T, getH(x[:6], beacons), R)
            path.append(x)
            (x, P) = predict(x, P, F, 0)

        state = agent.get_state().copy()
        state[3:6] = 0
        if i == 0:  # go up at 10 m/s
            state[4] = 10
        elif i == 1:  # go left at 12 m/s
            state[3] = -12
        elif i == 2:  # go down at 10 m/s
            state[4] = -10
        agent.update(state, None)

    print(f"Var: {np.max(P)}\n"
          f"Dist: {np.linalg.norm(x[:3] - agent.get_state()[:3])}\n")
    print(agent.get_state(), '\n\n', x)
    print(P)

    path = np.array(path)
    gt_path = np.array(gt_path)
    mapp(beacons, ax0, path, gt_path)
