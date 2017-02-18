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
freq = 280

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

axis_Lx = 0
axis_Ly = 0

wariai = 2

def uketori():
  global axis_Lx
  global axis_Ly
  host = '192.168.0.81'
  port = 4000
  bufsize = 512

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  with closing(sock):
    sock.bind((host, port))
    #print(sock.recv(bufsize))
    list= sock.recv(bufsize).split(",")
    #print list
    axis_Lx = float(list[0])
    axis_Ly = float(list[1])
    #print axis_Lx
    #print axis_Ly
  return

while True:
    #recv_axis
    uketori()
    #keisan
    if axis_Ly <= 0 :
        wheel_L = abs(axis_Ly) + axis_Lx
        wheel_R = abs(axis_Ly) - axis_Lx
    elif axis_Ly > 0 :
        wheel_L = - axis_Ly + axis_Lx
        wheel_R = - axis_Ly - axis_Lx
    else :
        print "o-no-"
        exit()
    #power save
    wheel_L = wheel_L * 75
    wheel_R = wheel_R * 75
    print "L=%f" % (wheel_L)
    print "R=%f" % (wheel_R)
    #CW or CCW
    if wheel_L > 0 :
        duty3 = wheel_L
        duty4 = 0
    else :
        duty3 = 0
        duty4 = abs(wheel_L)
    if wheel_R > 0 :
        duty1 = wheel_R
        duty2 = 0
    else :
        duty1 = 0
        duty2 = abs(wheel_R)
    p1.ChangeDutyCycle(duty1)
    p2.ChangeDutyCycle(duty2)
    p3.ChangeDutyCycle(duty3)
    p4.ChangeDutyCycle(duty4)

    time.sleep(0.1)
