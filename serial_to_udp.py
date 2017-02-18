# -*- coding: utf-8 -*-
import serial
import time
import socket

host = '127.0.0.1'
port = 3333

def okuru(mozi) :
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(mozi,(host, port))
    print mozi
    #time.sleep(0.01)

try :
    while(1):
        ser = serial.Serial('/dev/ttyACM0', 9600)
        moziretu = ser.readline()
        #print moziretu
        okuru(moziretu)

finally :
    ser = serial.Serial('/dev/ttyACM0', 9600)
    ser.close()
