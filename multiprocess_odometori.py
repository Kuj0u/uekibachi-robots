# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import math
import datetime
import serial
import socket
from multiprocessing import Process, Value, Array

#GPIO_PIN_NO
GPIO_ENC_LA = 26
GPIO_ENC_LB = 19
GPIO_ENC_RA = 20
GPIO_ENC_RB = 16

#counter
count_L = Value("i", 0)
count_R = Value("i", 0)
flag_pin_L = Value("i", 0)
flag_pin_R = Value("i", 0)
new_LA = Value("i", 0)
new_LB = Value("i", 0)
new_RA = Value("i", 0)
new_RB = Value("i", 0)
old_LA = Value("i", 0)
old_LB = Value("i", 0)
old_RA = Value("i", 0)
old_RB = Value("i", 0)
count_L_add = Value("i", 0)
count_R_add = Value("i", 0)
gyaku = -1

#keisann_you
Enc_P = 800.0
Gear = 2.0
Wheel_W = 61.5 / 1000.0 # メートル表記
Tread = 288.0 / 1000.0 # メートル表記
time_old = Value("d", time.time())
time_interval = 0.1
time_interval_dt = Value("d", 0)
kakusokudo_old = Value("d", 0)
shisei_old = Value("d", 0)
zahyou_x_old = Value("d", 0)
zahyou_y_old = Value("d", 0)
sokudo_old = Value("d", 0)
sokudo_L = Value("d", 0)
sokudo_R = Value("d", 0)

#speed control
Target_V_l = 1.0
Target_V_R = 1.0

#odometori file set
date_now = datetime.datetime.today()
fmt_filename = "log_odometori_" + str(date_now.strftime("%Y-%m-%d_%H:%M:%S")) + ".txt"
print fmt_filename
odometori_file_w = open(fmt_filename, "w")
odometori_file_a = open(fmt_filename, "a")
odometori_file_w.close()

#serial
ser = serial.Serial("/dev/ttyACM0", 9600)
light = Value("d", 0)
soil_water = Value("d", 0)
hot = Value("d", 0)
humidity = Value("d", 0)
water_supply = Value("d", 0)

#UDP
host = "127.0.0.1"
port = 3333
bufsize = 512

def udp_recv():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    udp_list = sock.recv(bufsize).split("\t")
    return udp_list

def serial_USB():
    list_val = udp_recv()
    if (list_val[0] == "start" and list_val[7] == "\n") :
        light.value = (float(list_val[1]) + float(list_val[2]) + float(list_val[3])) / 3.0
        soil_water.value = float(list_val[4])
        hot.value = float(list_val[5])
        humidity.value = float(list_val[6])

# encoder count bimyo
def enc_count_L(pin) :
    #global count_L, flag_pin_L, new_LA, new_LB, old_LA, old_LB, count_L_add
    volt = GPIO.input(pin)
    if(GPIO_ENC_LA == pin) :
        new_LA.value = volt
    else :
        new_LB.value = volt
    if(flag_pin_L.value == pin) :
        #print "逆回転だぜ！"
        count_L_add.value *= -1
    elif(old_LA.value == old_LB.value) :
        if(old_LA.value != new_LA.value) :
            count_L_add.value = 1
        else :
            count_L_add.value = -1
    else :
        if(old_LA.value == new_LA.value) :
            count_L_add.value = 1
        else :
            count_L_add.value = -1
    count_L.value += count_L_add.value * gyaku
    old_LA.value = new_LA.value
    old_LB.value = new_LB.value
    flag_pin_L.value = pin

# encoder count bimyo
def enc_count_R(pin) :
    #global count_R, flag_pin_R, new_RA, new_RB, old_RA, old_RB, count_R_add
    volt = GPIO.input(pin)
    if(GPIO_ENC_RA == pin) :
        new_RA.value = volt
    else :
        new_RB.value = volt
    if(flag_pin_R.value == pin) :
        #print "逆回転だぜ！"
        count_R_add.value *= -1
    elif(old_RA.value == old_RB.value) :
        if(old_RA.value != new_RA.value) :
            count_R_add.value = 1
        else :
            count_R_add.value = -1
    else :
        if(old_RA.value == new_RA.value) :
            count_R_add.value = 1
        else :
            count_R_add.value = -1
    count_R.value += count_R_add.value
    old_RA.value = new_RA.value
    old_RB.value = new_RB.value
    flag_pin_R.value = pin


def keisan() :
    #global time_old, count_L, count_R, kakusokudo_old, shisei_old, zahyou_x_old, zahyou_y_old, sokudo_old, sokudo_L, sokudo_R
    #time_now = time.time()
    if 1 == 1 :
        #time_interval_dt = time_now - time_old
        kakudo_L = (2.0 * math.pi * count_L.value) / Enc_P
        kakudo_R = (2.0 * math.pi * count_R.value) / Enc_P
        #早めにcount = 0にする
        count_L.value = 0
        count_R.value = 0
        kakusokudo_L = kakudo_L / Gear / time_interval_dt.value
        kakusokudo_R = kakudo_R / Gear / time_interval_dt.value
        sokudo_L.value = kakusokudo_L * Wheel_W
        sokudo_R.value = kakusokudo_R * Wheel_W
        sokudo = (sokudo_R.value + sokudo_L.value) / 2.0
        kakusokudo = (kakusokudo_R - kakusokudo_L) * Wheel_W / Tread
        shisei = (kakusokudo + kakusokudo_old.value) * time_interval_dt.value / 2.0 + shisei_old.value
        zahyou_x = (sokudo * math.cos(shisei) + sokudo_old.value * math.cos(shisei_old.value)) * time_interval_dt.value / 2.0 + zahyou_x_old.value
        zahyou_y = (sokudo * math.sin(shisei) + sokudo_old.value * math.sin(shisei_old.value)) * time_interval_dt.value / 2.0 + zahyou_y_old.value
        #kokokara print
        #print "速度  : " + str(sokudo)
        #print "座標x : " + str(zahyou_x)
        #print "座標y : " + str(zahyou_y)
        #print "姿勢  : " + str(shisei / math.pi * 180) + " deg  ||  " +str(shisei) + " rad"
        print "明るさ%lf" % light.value
        print "-----"
        file_add = ("%lf\t%lf\t%lf\t%d\t%d\t%lf\t%lf\t%lf\n") % (zahyou_x, zahyou_y, shisei, light.value, soil_water.value, hot.value, humidity.value, water_supply.value)
        odometori_file_a.write(file_add)
        #odometori_file_a.close()
        kakusokudo_old.value = kakusokudo
        sokudo_old.value = sokudo
        shisei_old.value = shisei
        zahyou_x_old.value = zahyou_x
        zahyou_y_old.value = zahyou_y



def omuron_R():
    GPIO.setup(GPIO_ENC_RA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(GPIO_ENC_RB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(GPIO_ENC_RA, GPIO.BOTH, enc_count_R)
    GPIO.add_event_detect(GPIO_ENC_RB, GPIO.BOTH, enc_count_R)
    while True:
        time.sleep(1)

def omuron_L():
    GPIO.setup(GPIO_ENC_LA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(GPIO_ENC_LB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(GPIO_ENC_LA, GPIO.BOTH, enc_count_L)
    GPIO.add_event_detect(GPIO_ENC_LB, GPIO.BOTH, enc_count_L)
    while True :
        time.sleep(1)

def sensa_read():
    print"センサstart"
    while True:
        serial_USB()
        time.sleep(0.01)

def odo_keisan():
    while True:
        time_now = time.time()
        time_interval_dt.value = time_now - time_old.value
        if time_interval < time_interval_dt.value :
            keisan()
            time_old.value = time_now
        else :
            time.sleep(0.01)

try:
    GPIO.setmode(GPIO.BCM)
    enc_count_L_process = Process(name="enc_L", target=omuron_L)
    enc_count_R_process = Process(name="enc_R", target=omuron_R)
    sensa_process = Process(name="sensa", target=sensa_read)
    keisan_process = Process(name="keisan", target=odo_keisan)
    enc_count_L_process.start()
    enc_count_R_process.start()
    sensa_process.start()
    keisan_process.start()
    while True :
        time.sleep(1)

finally :
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.close()
    GPIO.cleanup()
    odometori_file_a.close()
    print "おわり"

