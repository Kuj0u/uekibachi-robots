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

power = 99.9

def uketori():
  a = 0
  b = 0
  c = 0
  global axis_Ly
  global axis_Ry
  host = '192.168.0.222'
  port = 12345
  bufsize = 2048


  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  with closing(sock):
    sock.bind((host, port))
    #print(sock.recv(bufsize))
    list= sock.recv(bufsize).split(",")
    #print list
    if str(list[0]) is str(0) :
        axis_Ry = float(list[1]) * power
        axis_Ly = float(list[1]) * power
    elif str(list[0]) is str(1) :
        axis_Ry = float(list[1]) * power * -1
        axis_Ly = float(list[1]) * power
    elif str(list[0]) is str(2) :
        axis_Ry = float(list[1]) * power
        axis_Ly = float(list[1]) * power * -1
    print list[0]
    print list[1]
    print "hanndann" + str(list[0])
    print "hidari" + str(axis_Ly)
    print "migi" + str(axis_Ry)
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

