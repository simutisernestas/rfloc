"""Variance data collection
 
 Run indefinitely, until got exit input:
    1) take in distance
    2) run until collected N samples
    3) put in a csv file for instance, column name is GT distance, rows are measured values
       or just new file per one GT distance and write measurements there, not to fuck up 
       a single file in case of any errors : )
"""

from tomlkit import value
from getdata import read_measurements
import time
import pickle
import numpy as np

# TBC config
N_TO_COLLECT = 30 
DEVICE_POS = np.array([4.523, 5.134, .503])

if __name__ == "__main__":
    timeout = 10
    start = time.time()
    beacon = None
    while (time.time() - start) < timeout:
        # update from measurements
        devices = read_measurements()
        if len(devices) == 0:
            print("Can't detect an active beacon...")
            continue
        beacon = list(devices.keys())[0]
        print(f"Found beacon: {beacon}. Starting variance estimation...")
        break

    while True:
        gt_pos = input('Provide ground truth distance: ')
        try:
            values = [float(x) for x in gt_pos.split()]
            if len(values) != 3:
                raise Exception
            gt_pos = np.array(values)
        except:
            print("Failed while reading GT position, exiting : )")
            exit()

        measurements = []
        for _ in range(N_TO_COLLECT*2):
            if len(measurements) == N_TO_COLLECT:
                break
            devices = read_measurements()
            if len(devices) == 0:
                # print("Didn't get any measurements form read..")
                continue
            dist = devices[beacon]["Range"]
            measurements.append(dist)
            if len(measurements) % 10 == 0:
                print(f"Collected {len(measurements)} so far : )")
            if len(measurements) >= N_TO_COLLECT:
                break

        if len(measurements) < N_TO_COLLECT:
            print(f"Less than {N_TO_COLLECT} measurements..."
                  f"Skipping saving...")
            continue

        gt_dist = np.linalg.norm(gt_pos - DEVICE_POS, 2)
        with open(f'data/{beacon}_{gt_dist}_dist.pkl', 'wb') as f:
            pickle.dump(measurements, f, pickle.HIGHEST_PROTOCOL)
