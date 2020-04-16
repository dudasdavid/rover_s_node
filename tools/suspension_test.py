#! /usr/bin/env python3

import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.close()
ser.open()
ser.isOpen()

ser.write(b"SRE\r\n")
time.sleep(1)

speed = 0.01

for j in range(0,4):

    for i in range(0,101):
        FL = 100-i
        FR = 100-i
        RL = 100
        RR = 100
        command = "SUM%03d%03d%03d%03d" % (FL, FR, RL, RR)
        ser.write(b"%s\r\n" % command.encode('ascii','ignore'))
        time.sleep(speed)

    for i in range(0,101):
        FL = i
        FR = i
        RL = 100-i
        RR = 100-i
        command = "SUM%03d%03d%03d%03d" % (FL, FR, RL, RR)
        ser.write(b"%s\r\n" % command.encode('ascii','ignore'))
        time.sleep(speed)

    for i in range(0,101):
        FL = 100-i
        FR = 100
        RL = 0
        RR = i
        command = "SUM%03d%03d%03d%03d" % (FL, FR, RL, RR)
        ser.write(b"%s\r\n" % command.encode('ascii','ignore'))
        time.sleep(speed)

    for i in range(0, 101):
        FL = i
        FR = 100-i
        RL = i
        RR = 100-i
        command = "SUM%03d%03d%03d%03d" % (FL, FR, RL, RR)
        ser.write(b"%s\r\n" % command.encode('ascii', 'ignore'))
        time.sleep(speed)

    for i in range(0, 101):
        FL = 100-i
        FR = 0
        RL = 100-i
        RR = 0
        command = "SUM%03d%03d%03d%03d" % (FL, FR, RL, RR)
        ser.write(b"%s\r\n" % command.encode('ascii', 'ignore'))
        time.sleep(speed)

    for i in range(0, 101):
        FL = i
        FR = i
        RL = i
        RR = i
        command = "SUM%03d%03d%03d%03d" % (FL, FR, RL, RR)
        ser.write(b"%s\r\n" % command.encode('ascii', 'ignore'))
        time.sleep(speed)

ser.close()
exit()
