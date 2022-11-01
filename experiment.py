import numpy as np
from rfloc import *
from bagpy import bagreader
import pandas as pd

RANGE_MEAS_LIFESPAN = .2
BEACON_MAP = {
    "20725": Beacon("20725", np.array([-1.51, 3.32, 1.05]), lifespan=RANGE_MEAS_LIFESPAN),
    "52422": Beacon("52422", np.array([4.22, 2.38, .7]), lifespan=RANGE_MEAS_LIFESPAN),
    "43209": Beacon("43209", np.array([2.51, -6.11, .009]), lifespan=RANGE_MEAS_LIFESPAN),
    "27570": Beacon("27570", np.array([-3.99, -6.07, .41]), lifespan=RANGE_MEAS_LIFESPAN),
}
BAG_NAME = "2022-10-31-15-05-16.bag"

if __name__ == '__main__':
    DIM_X = 6
    DIM_Z = 4
    Pnoise = 3e-4
    R = np.diag([Pnoise]*DIM_Z)
    P = np.eye(DIM_X)
    P *= 10

    fdev = 2.0
    path = []
    gt_path = []

    agent = Agent()
    ax0 = np.zeros((DIM_X, 1))
    b = bagreader(BAG_NAME)
    pose_csv = b.message_by_topic("/vrpn_client_node/awww/pose")
    df = pd.read_csv(pose_csv)
    df = df.reset_index()
    for index, row in df.iterrows():
        ax0[0] = row["pose.position.x"]
        ax0[1] = row["pose.position.y"]
        ax0[2] = row["pose.position.z"]
        stamp = row["Time"]
        agent.update(ax0, stamp)
        print(f"{stamp}; Agent x0: {ax0[:3].T}")
        break
    gtlen = len(df["pose.position.x"])
    gt_path = np.array(
        [df["pose.position.x"], df["pose.position.y"], df["pose.position.z"]]).T
    gt_path = gt_path[:int(gtlen/fdev), :]

    b = bagreader(BAG_NAME)
    chatter_csv = b.message_by_topic("/chatter")
    df = pd.read_csv(chatter_csv)
    df = df.reset_index()
    path = []
    for index, row in df.iterrows():
        beacon_id = str(int(row["header.frame_id"]))
        stamp = row["Time"]
        meas_range = row["range"]
        if beacon_id not in BEACON_MAP:
            print("NOT IN THE MAP! : O")
            continue
        BEACON_MAP[beacon_id].update_range(meas_range, stamp)
        # print(f"{stamp}; Beacon: {beacon_id}; Range {meas_range}")
        measurements = [v.get_range(stamp) for k, v in BEACON_MAP.items()]
        z = []
        idxs = []
        for i,m in enumerate(measurements):
            if m is not None:
                z.append(m)
                idxs.append(i)
        # got_4_meas = True if None not in measurements else False
        # if got_4_meas:
        last_update = agent.get_last_update()
        dt = stamp - last_update
        if (len(z) > 2 and dt > RANGE_MEAS_LIFESPAN) or len(z) == 4:
            obs_beacons = [list(BEACON_MAP.values())[i] for i in idxs]
            for b in obs_beacons:
                b.discard_meas()
            x = agent.get_state()
            (x, P) = predict(x, P, getF(dt), 0, Pnoise)
            z = np.array(z).reshape((len(z), 1))
            if (abs(z) > 10).any():
                continue
            # (x, P) = update(x, hx(x, BEACON_MAP.values()),
            #                 P, z, getH(x, BEACON_MAP.values()), R)
            (x, P) = update(x, hx(x, obs_beacons),
                            P, z, getH(x, obs_beacons), np.diag([Pnoise]*len(z)))
            path.append(x)
            agent.update(x, stamp)

    print(f"Var: {np.max(P)}\n"
          f"Pos: {agent.get_state()[:3].T}")

    path = np.array(path)
    pathlen = path.shape[0]
    path = path[:int(pathlen/fdev), :]

    mapp(BEACON_MAP.values(), ax0, path, gt_path, threed=True)
