# -*- coding: utf-8 -*-
import serial
import time

while(1):
    ser = serial.Serial('/dev/ttyACM0', 9600)
    #time.sleep(1)
    print(ser.readline())
ser.close()
