import RPi.GPIO as GPIO
import time
import math
import socket
from sys import exit
from contextlib import closing

a = 23
b = 24
c = 25
d = 11
freq = 250
#seigen = 80.0

GPIO.setmode(GPIO.BCM)
GPIO.setup(a,GPIO.OUT)
GPIO.setup(b,GPIO.OUT)
GPIO.setup(c,GPIO.OUT)
GPIO.setup(d,GPIO.OUT)

p1 = GPIO.PWM(a,freq)
p1.start(0)
p2 = GPIO.PWM(b,freq)
p2.start(0)
p3 = GPIO.PWM(c,freq)
p3.start(0)
p4 = GPIO.PWM(d,freq)
p4.start(0)

axis_Ly = 0
axis_Ry = 0

def uketori():
  global axis_Ly
  global axis_Ry
  host = '192.168.0.81'
  port = 4000
  bufsize = 4096

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  with closing(sock):
    sock.bind((host, port))
    #print(sock.recv(bufsize))
    list= sock.recv(bufsize).split(",")
    #print list
    axis_Ly = float(list[1]) * 100.0
    axis_Ry = float(list[3]) * 100.0
    print axis_Ly
    print axis_Ry
  return


while True:
    uketori()
    if axis_Ly >= 0:
        duty3 = 0
        #duty4 = axis_xL*seigen
        duty4 = axis_Ly
    elif axis_Ly < 0:
        #duty3 = abs(axis_xL)*seigen
        duty3 = abs(axis_Ly)
        duty4 = 0
    if axis_Ry >= 0:
        duty1 = 0
        #duty2 = axis_xR*seigen
        duty2 = axis_Ry
    elif axis_Ry < 0:
        #duty1 = abs(axis_xR)*seigen
        duty1 = abs(axis_Ry)
        duty2 = 0
    p1.ChangeDutyCycle(duty1)
    p2.ChangeDutyCycle(duty2)
    p3.ChangeDutyCycle(duty3)
    p4.ChangeDutyCycle(duty4)
    time.sleep(0.1)

