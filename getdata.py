import serial
import time

DUMMY = False

try:
    ser = serial.Serial('/dev/tty.usbserial-02312F0F', 115200, timeout=.01)
except:
    print("DUMMY DATA!")
    DUMMY = True
    pass


def read_measurements(timeout=.1, show=False, dummy=False):
    """ {beacon: {time [ms], range [m], RXpower [dBm]}}
    {'6022': {'ts': 101.0, 'Range': 1.42, 'RXpower': -85.21}}"""
    if dummy or DUMMY:
        return {'6022': {'ts': 101.0, 'Range': 7.0710678118654755, 'RXpower': -85.21},
                '6023': {'ts': 101.0, 'Range': 7.0710678118654755, 'RXpower': -85.21},
                '6024': {'ts': 101.0, 'Range': 7.0710678118654755, 'RXpower': -85.21},
                '6025': {'ts': 101.0, 'Range': 7.0710678118654755, 'RXpower': -85.21}}
    devices = {}
    readOut = 0
    start = time.time()
    while time.time() - start < timeout:
        try:
            readOut = ser.readline().decode('ascii')
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
        except Exception as e:
            print(f"Something went wrong: {e}")
            continue

    ser.flush()  # flush the buffer
    return devices


if __name__ == '__main__':
    try:
        while True:
            read_measurements(show=True)
    except KeyboardInterrupt:
        exit()
