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

def uketori():
  global axis_Lx
  global axis_Ly
  host = '192.168.0.222'
  port = 4000
  bufsize = 4096

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  with closing(sock):
    sock.bind((host, port))
    list= sock.recv(bufsize).split(",")
    axis_Lx = float(list[0])
    axis_Ly = float(list[1])
  return

while True:
    uketori()
    power_wheel = axis_Ly * -1
    if axis_Lx < 0 :
        wariai_L = 1.0 - abs(axis_Lx)
        wariai_R = 1.0
    else :
        wariai_L = 1.0
        wariai_R = 1.0 - abs(axis_Lx)
    wheel_L = power_wheel * wariai_L
    wheel_R = power_wheel * wariai_R
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
