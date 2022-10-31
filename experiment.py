import numpy as np
import matplotlib.pyplot as plt
from getdata import read_measurements
import time
from rfloc import *
from bagpy import bagreader
import pandas as pd

BEACON_MAP = {
    "11597": Beacon("11597", np.array([10, 1, .4])),
    "12962": Beacon("12962", np.array([-10, 1, .3])),
    "53690": Beacon("53690", np.array([-20, -1, .2])),
    "23923": Beacon("23923", np.array([1, -1, .1])),
}


# TODO: unused
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
    DIM_X = 6
    DIM_Z = 4
    path = []
    gt_path = []
    # initialize filter
    range_std = 1  # meters
    R = np.diag([range_std**2]*4)
    P = np.eye(DIM_X)
    P *= 50

    ax0 = np.zeros((6, 1))
    # b = bagreader("pose.bag")
    # pose_csv = b.message_by_topic("/vrpn_client_node/awww/pose")
    # df = pd.read_csv(pose_csv)
    # df = df.reset_index()
    # for index, row in df.iterrows():
    #     ax0[0] = int(row["position.x"])
    #     ax0[1] = int(row["position.y"])
    #     ax0[2] = int(row["position.z"])
    #     stamp = row["Time"]
    #     print(f"{stamp}; Agent x0: {ax0}")
    #     break
    agent = Agent(x0=ax0)

    b = bagreader("2022-10-31-11-50-47.bag")
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
        print(f"{stamp}; Beacon: {beacon_id}; Range {meas_range}")
        measurements = [v.get_range(stamp) for k, v in BEACON_MAP.items()]
        got_4_meas = True if None not in measurements else False
        if got_4_meas:
            x = agent.get_state()
            last_update = agent.get_last_update()
            dt = stamp - last_update
            (x, P) = predict(x, P, getF(dt), 0)
            z = np.array(measurements).reshape((4, 1))
            (x, P) = update(x, hx(x, BEACON_MAP.values()),
                            P, z, getH(x, BEACON_MAP.values()), R)
            path.append(x)
            agent.update(x, stamp)

    print(f"Var: {np.max(P)}\n"
          f"Pos: {agent.get_state()[:3].T}\n")

    path = np.array(path)
    gt_path = np.array(gt_path)
    mapp(BEACON_MAP.values(), ax0, path)  # gt_path
