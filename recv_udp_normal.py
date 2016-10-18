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

axis_x = 0
axis_y = 0

def uketori():
  global axis_x
  global axis_y
  host = '192.168.0.81'
  port = 4000
  bufsize = 4096

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  with closing(sock):
    sock.bind((host, port))
    #print(sock.recv(bufsize))
    list= sock.recv(bufsize).split(",")
    #print list
    axis_x = float(list[0])
    axis_y = float(list[1])
    print axis_x
    print axis_y
  return

while True:
    uketori()
    if axis_y >= 0:
        duty1 = 0
        duty2 = axis_y*50.0
        duty3 = 0
        duty4 = axis_y*50.0
        if axis_x >= 0:
            duty4 = duty4 + axis_x*50.0
        elif axis_x < 0:
            duty2 = duty2 + abs(axis_x)*50.0
        #print "L:%d"%duty4
        #print "R:%d"%duty2
    elif axis_y < 0:
        duty1 = abs(axis_y)*50.0
        duty2 = 0
        duty3 = abs(axis_y)*50.0
        duty4 = 0
        if axis_x >= 0:
            duty3 = duty3 + axis_x*50.0
        elif axis_x < 0:
            duty1 = duty1 + abs(axis_x)*50.0
        #print "L:%d"%duty3
        #print "R:%d"%duty1
    p1.ChangeDutyCycle(duty1)
    p2.ChangeDutyCycle(duty2)
    p3.ChangeDutyCycle(duty3)
    p4.ChangeDutyCycle(duty4)

    time.sleep(0.1)
