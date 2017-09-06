# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import math
import datetime
import serial
import socket

log_file_name = 'log_odometori.txt'
flag = 0
status_step_now = 0
move_comp = 0

encoderdata = serial.Serial('/dev/ttyACM0',230400, timeout = 1000)
countdata = 0
count_L = 0
count_R = 0
count_L_add = 0
count_R_add = 0
Enc_P = 800.0  #200パルスの四逓倍
Gear = 2.0  #ギア比
Wheel_W = 100.0 / 1000.0 # メートル表記 車輪の半径
Tread = 356.6 / 1000.0 # メートル表記　車輪間距離
Enc_P = 800.0
Gear = 2.0
Wheel_W = 62.8 / 1000.0 # メートル表記
Tread = 288.0 / 1000.0 # メートル表記
time_old = time.time()
time_interval = 0.1
time_interval_dt = 0
kakusokudo_old = 0
shisei_old = 0
zahyou_x_old = 0
zahyou_y_old = 0
sokudo_old = 0
sokudo_Wheel_L = 0
sokudo_Wheel_R = 0
sokudo_t = 0
sokudo_Wheel_L_t0 = 0
sokudo_Wheel_L_t1 = 0
sokudo_Wheel_L_t2 = 0
sokudo_Wheel_R_t0 = 0
sokudo_Wheel_R_t1 = 0
sokudo_Wheel_R_t2 = 0

Moter_R1_Pin = 23
Moter_R2_Pin = 24
Moter_L1_Pin = 25
Moter_L2_Pin = 11
PWM_freq = 250.0
PWM_power = 100.0
speed_MAX = 10.8
pwm_power_L = 0
PWM_power_R = 0

run_mode = 1

def motor_stop():
    time.sleep(0.001)

def enc_count_L() :
    global count_L, count_R, countdata, count_L_add, count_R_add
    countdata = int(encoderdata.readline())
    count_L_add = (3 & countdata) - 2
    count_R_add = (3 & (countdata / 100)) - 2
    count_L += count_L_add
    count_R += count_R_add
    keisan()


def keisan() :
    global time_old, count_L, count_R, kakusokudo_old, shisei_old, zahyou_x_old, zahyou_y_old, sokudo_old, sokudo_Wheel_L, sokudo_Wheel_R, sokudo_Wheel_L_t0, sokudo_Wheel_L_t1, sokudo_Wheel_L_t2, sokudo_Wheel_R_t0, sokudo_Wheel_R_t1, sokudo_Wheel_R_t2, now_distance
    time_now = time.time()
    if(time_now - time_old > time_interval) :
        time_interval_dt = time_now - time_old
        kakudo_L = (2.0 * math.pi * count_L) / Enc_P
        kakudo_R = (2.0 * math.pi * count_R) / Enc_P
        kakusokudo_L = kakudo_L / Gear / time_interval_dt
        kakusokudo_R = kakudo_R / Gear / time_interval_dt
        sokudo_Wheel_L = kakusokudo_L * Wheel_W
        sokudo_Wheel_R = kakusokudo_R * Wheel_W
        sokudo = (sokudo_Wheel_L + sokudo_Wheel_R) / 2.0
        kakusokudo = (kakusokudo_R - kakusokudo_L) * Wheel_W / Tread
        shisei = (kakusokudo + kakusokudo_old) * time_interval_dt / 2.0 + shisei_old
        zahyou_x = (sokudo * math.cos(shisei) + sokudo_old * math.cos(shisei_old)) * time_interval_dt / 2.0 + zahyou_x_old
        zahyou_y = (sokudo * math.sin(shisei) + sokudo_old * math.sin(shisei_old)) * time_interval_dt / 2.0 + zahyou_y_old
        file_add = str(zahyou_x) + "\t" + str(zahyou_y) + "\t" + str(shisei) + "\n"
        odometori_file_a.write(file_add)
        #odometori_file_a.close()
        count_L = 0
        count_R = 0
        time_old = time_now
        kakusokudo_old = kakusokudo
        sokudo_Wheel_L_t2 = sokudo_Wheel_L_t1
        sokudo_Wheel_L_t1 = sokudo_Wheel_L_t0
        sokudo_Wheel_L_t0 = sokudo_Wheel_L
        sokudo_Wheel_R_t2 = sokudo_Wheel_R_t1
        sokudo_Wheel_R_t1 = sokudo_Wheel_R_t0
        sokudo_Wheel_R_t0 = sokudo_Wheel_R
        sokudo_old = sokudo
        shisei_old = shisei
        now_distance += math.sqrt((zahyou_x - zahyou_x_old) * (zahyou_x - zahyou_x_old) + (zahyou_y - zahyou_y_old) * (zahyou_y - zahyou_y_old))
        zahyou_x_old = zahyou_x
        zahyou_y_old = zahyou_y


# GPIO setup
GPIO.setmode(GPIO.BCM)

#setup moter PWM
GPIO.setup(Moter_L1_Pin, GPIO.OUT)
GPIO.setup(Moter_L2_Pin, GPIO.OUT)
GPIO.setup(Moter_R1_Pin, GPIO.OUT)
GPIO.setup(Moter_R2_Pin, GPIO.OUT)
Moter_L1_PWM = GPIO.PWM(Moter_L1_Pin, PWM_freq)
Moter_L2_PWM = GPIO.PWM(Moter_L2_Pin, PWM_freq)
Moter_R1_PWM = GPIO.PWM(Moter_R1_Pin, PWM_freq)
Moter_R2_PWM = GPIO.PWM(Moter_R2_Pin, PWM_freq)
Moter_L1_PWM.start(0)
Moter_L2_PWM.start(0)
Moter_R1_PWM.start(0)
Moter_R2_PWM.start(0)

# event wait
try:
    while True:
        enc_count_L()
        time.sleep(0.001)

finally :
    print "END"
    motor_stop()
    GPIO.cleanup()
    fileopen.close()
