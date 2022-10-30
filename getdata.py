import serial
import time
import numpy as np
from sys import platform

DUMMY = False

try:
    if platform == "linux" or platform == "linux2":
        ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=.01)
    elif platform == "win32":
        ser = serial.Serial('COM8', 115200, timeout=.01)
except:
    print("Failed to open serial port... Fedding DUMMY DATA!")
    DUMMY = True
    pass


def flush(serial):
    # flush the buffer
    serial.reset_input_buffer()
    serial.reset_output_buffer()
    serial.flush()


def read_measurements(timeout=.1, show=False, dummy=False):
    """ {beacon: {time [ms], range [m], RXpower [dBm]}}
    {'6022': {'ts': 101.0, 'Range': 1.42, 'RXpower': -85.21}}"""
    if dummy or DUMMY:
        r = 7.0710678118654755 + np.random.random()
        return {'6033': {'ts': 101.0, 'Range': r, 'RXpower': -85.21},
                '6023': {'ts': 101.0, 'Range': r, 'RXpower': -85.21},
                '6024': {'ts': 101.0, 'Range': r, 'RXpower': -85.21},
                '6025': {'ts': 101.0, 'Range': r, 'RXpower': -85.21}}
    flush(ser)

    devices = {}
    readOut = 0
    start = time.time()
    while time.time() - start < timeout:
        # try:
        readOut = ser.readline().decode('ascii')
        if show and readOut != '':
            print(readOut)
        if "Range" not in readOut:
            continue
        readOut = readOut.split('\t')
        parsed = {}
        device = ""
        for p in readOut:
            p = p.strip()
            p = p.replace(":", "")
            (field, value) = p.split()
            if field != "from":
                value = float(value)
                parsed[field] = value
            else:
                device = value
        devices[device] = parsed
        if show:
            print(devices)
        # except Exception as e:
        #     print(f"Something went wrong: {e}")
        #     continue
    return devices


if __name__ == '__main__':
    try:
        while True:
            read_measurements(show=True)
    except KeyboardInterrupt:
        exit()
