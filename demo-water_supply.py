import RPi.GPIO as GPIO
import time
import math
import serial

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

speed = 60

suty1 = 0
duty2 = 0
duty3 = 0
duty4 = 0

def ugoki():
    p1.ChangeDutyCycle(duty1)
    p2.ChangeDutyCycle(duty2)
    p3.ChangeDutyCycle(duty3)
    p4.ChangeDutyCycle(duty4)

while True:
    time.sleep(3)
    duty1 = speed
    duty3 = speed - 3
    print("1")
    ugoki()
    time.sleep(2)
    duty1 = 0
    duty3 = 0
    print("2")
    ugoki()
    time.sleep(10)
    duty2 = speed
    duty4 = speed -3
    print("3")
    ugoki()
    time.sleep(3)
    duty2 = 0
    duty4 = 0
    print("4")
    ugoki()
    time.sleep(10)

