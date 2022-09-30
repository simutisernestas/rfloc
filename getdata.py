import serial
import time

ser = serial.Serial('/dev/tty.usbserial-02312F0F', 115200, timeout=1)

readOut = 0  # chars waiting from laser range finder

print("Starting up")
connected = False
commandToSend = 1  # get the distance in mm

while True:
    # print("Writing: ",  commandToSend)
    ser.write(str(commandToSend).encode())
    while True:
        try:
            # print("Attempt to Read")
            readOut = ser.readline().decode('ascii')
            time.sleep(.01)
            if readOut != "":
                print("Reading: ", readOut)
            break
        except KeyboardInterrupt:
            exit()
    ser.flush() # flush the buffer


