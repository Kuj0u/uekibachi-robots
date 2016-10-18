#coding: UTF-8
import RPi.GPIO as GPIO
import time
import math
import serial

a = 23
b = 24
c = 25
d = 11

GPIO.setmode(GPIO.BCM)
GPIO.setup(a,GPIO.OUT)
GPIO.setup(b,GPIO.OUT)
GPIO.setup(c,GPIO.OUT)
GPIO.setup(d,GPIO.OUT)

GPIO.output(a, False)
GPIO.output(b, False)
GPIO.output(c, False)
GPIO.output(d, False)

while True:
    ser = serial.Serial('/dev/ttyACM0', 9600)
    akarusa = ser.readline()
    hikari = int(akarusa)
    print(hikari)
    #time.sleep(2)
    if(hikari > 600):
        GPIO.output(a, False)
        GPIO.output(b, False)
        GPIO.output(c, False)
        GPIO.output(d, False)
    else:
        GPIO.output(a, True)
        GPIO.output(c, True)

