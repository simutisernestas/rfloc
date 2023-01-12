import numpy as np
from rfloc import *
from bagpy import bagreader
import pandas as pd

RANGE_MEAS_LIFESPAN = 5e-2
NZ_FOR_UPDATE = 3
BEACON_MAP = {
    "20725": Beacon("20725", np.array([-1.51, 3.32, 1.05]), lifespan=RANGE_MEAS_LIFESPAN),
    "52422": Beacon("52422", np.array([4.22, 2.38, .7]), lifespan=RANGE_MEAS_LIFESPAN),
    "43209": Beacon("43209", np.array([2.51, -6.11, .009]), lifespan=RANGE_MEAS_LIFESPAN),
    "27570": Beacon("27570", np.array([-3.99, -6.07, .41]), lifespan=RANGE_MEAS_LIFESPAN),
}
BAG_NAME = "data/2022-10-31-15-05-16.bag"

if __name__ == '__main__':
    DIM_X = 6
    Pnoise = 3e-4*4
    P = np.eye(DIM_X)*5e-2

    fdev = 2.0
    path = []
    gt_path = []
    Pss = []
    innos = []

    agent = Agent()
    ax0 = np.zeros((DIM_X, 1))
    b = bagreader(BAG_NAME, verbose=False)
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
    gt_time = np.array(df["Time"])[:int(gtlen/fdev)]

    b = bagreader(BAG_NAME, verbose=False)
    chatter_csv = b.message_by_topic("/chatter")
    df = pd.read_csv(chatter_csv)
    df = df.reset_index()
    path = []
    path_time = []
    for index, row in df.iterrows():
        beacon_id = str(int(row["header.frame_id"]))
        stamp = row["Time"]
        meas_range = row["range"]
        if beacon_id not in BEACON_MAP:
            print("NOT IN THE MAP! : O")
            continue
        BEACON_MAP[beacon_id].update_range(meas_range, stamp)
        measurements = [v.get_range(stamp) for k, v in BEACON_MAP.items()]
        z = []
        idxs = []
        for i, m in enumerate(measurements):
            if m is not None:
                z.append(m)
                idxs.append(i)
        last_update = agent.get_last_update()
        dt = stamp - last_update
        x = agent.get_state()
        (x, P) = predict(x, P, getF(dt), 0, Pnoise)
        if len(z) >= NZ_FOR_UPDATE:
            obs_beacons = [list(BEACON_MAP.values())[i] for i in idxs]
            z = np.array(z).reshape((len(z), 1))
            if (abs(z) > 15).any():
                continue
            R = np.diag([Pnoise]*len(z))
            inno = z - hx(x, obs_beacons)
            innos.append(inno)
            (x, P) = update(x, hx(x, obs_beacons),
                            P, z, getH(x, obs_beacons), R)
        agent.update(x, stamp)
        Pss.append(P)
        path.append(x)
        path_time.append(stamp)

    print(f"Var: {np.max(P)}\n"
          f"Pos: {agent.get_state()[:3].T}")

    path = np.array(path)
    pathlen = path.shape[0]
    path = path[:int(pathlen/fdev), :]
    path_time = path_time[:int(pathlen/fdev)]
    Pss = Pss[:int(pathlen/fdev)]

    innos = np.array(innos)
    np.save("data/innos.npy", innos)
    np.save("data/path.npy", path)
    np.save("data/path_time.npy", path_time)
    np.save("data/gt_path.npy", gt_path)
    np.save("data/gt_time.npy", gt_time)

    # plot_gt_vs_data_with_cov(path, gt_path, Pss, BEACON_MAP.values())
    # mapp(BEACON_MAP.values(), ax0, path, gt_path, Pss, threed=True)
    # plt.show()
