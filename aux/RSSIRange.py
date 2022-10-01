import math
import os
import time

# https://stackoverflow.com/questions/11217674/how-to-calculate-distance-from-wifi-router-using-signal-strength
K = 27.55

# iwlist wlp4s0 channel | grep Current
# freqInMHz = 5.18 * 1e3

# iwconfig wlp4s0 | grep -i --color quality
# signalLevelInDb = -49

while True:
    levelstr = os.popen("iwconfig wlp4s0 | grep -i --color quality").read()
    signalLevelInDb = float(levelstr.split('=')[-1].split()[0])

    freqstr = os.popen("iwlist wlp4s0 channel | grep Current").read()
    freqInMHz = float(freqstr.split(':')[1].split()[0]) * 1e3

    exp = (K - (20 * math.log10(freqInMHz)) + abs(signalLevelInDb)) / 20.0
    dist = math.pow(10.0, exp)  # m
    print(f"Distance: {dist}; Signal Db: {signalLevelInDb}")

    time.sleep(1)