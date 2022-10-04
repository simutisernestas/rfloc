"""TODO: implement variance estimation experiment code!
 
 Run indefinitely, until got exit input:
    1) take in distance
    2) run until collected N samples
    3) put in a csv file for instance, column name is GT distance, rows are measured values
       or just new file per one GT distance and write measurements there, not to fuck up 
       a single file in case of any errors : )
"""

from getdata import read_measurements
import time
import pickle

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
        gt_d = input('Provide ground truth distance: ')
        try:
            gt_d = float(gt_d)
        except:
            print("GT distance was not float, exiting : )")
            exit()
        measurements = []
        for _ in range(120):
            if len(measurements) == 100:
                break
            devices = read_measurements()
            if len(devices) == 0:
                print("Didn't get any measurements form read..")
                continue
            dist = devices[beacon]["Range"]
            measurements.append(dist)
        with open(f'data/{beacon}_{gt_d}_dist.pkl', 'wb') as f:
            pickle.dump(measurements, f, pickle.HIGHEST_PROTOCOL)
