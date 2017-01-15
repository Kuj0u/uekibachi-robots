# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import math
import datetime

#GPIO_PIN_NO
GPIO_ENC_LA = 26
GPIO_ENC_LB = 19
GPIO_ENC_RA = 20
GPIO_ENC_RB = 16

#counter
count_L = 0
count_R = 0
flag_pin_L = 0
flag_pin_R = 0
new_LA = 0
new_LB = 0
new_RA = 0
new_RB = 0
old_LA = 0
old_LB = 0
old_RA = 0
old_RB = 0
count_L_add = 0
count_R_add = 0
gyaku = -1

#keisann_you
Enc_P = 800.0
Gear = 2.0
Wheel_W = 61.5 / 1000.0 # メートル表記
Tread = 288.0 / 1000.0 # メートル表記
time_old = time.time()
time_interval = 0.5
time_interval_dt = 0
kakusokudo_old = 0
shisei_old = 0
zahyou_x_old = 0
zahyou_y_old = 0
sokudo_old = 0
sokudo_L = 0
sokudo_R = 0

#speed control
Target_V_l = 2.0
Target_V_R = 3.0

#odometori file set
date_now = datetime.datetime.today()
fmt_filename = "log_odometori_" + str(date_now.strftime("%Y-%m-%d_%H:%M:%S")) + ".txt"
print fmt_filename
odometori_file_w = open(fmt_filename, "w")
odometori_file_a = open(fmt_filename, "a")
odometori_file_w.close()

# encoder count bimyo
def enc_count_L(pin) :
    global count_L, flag_pin_L, new_LA, new_LB, old_LA, old_LB, count_L_add
    volt = GPIO.input(pin)
    if(GPIO_ENC_LA == pin) :
        new_LA = volt
    else :
        new_LB = volt
    if(flag_pin_L == pin) :
        #print "逆回転だぜ！"
        count_L_add *= -1
    elif(old_LA == old_LB) :
        if(old_LA != new_LA) :
            count_L_add = 1
        else :
            count_L_add = -1
    else :
        if(old_LA == new_LA) :
            count_L_add = 1
        else :
            count_L_add = -1
    count_L += count_L_add * gyaku
    old_LA = new_LA
    old_LB = new_LB
    flag_pin_L = pin
    keisan()

# encoder count bimyo
def enc_count_R(pin) :
    global count_R, flag_pin_R, new_RA, new_RB, old_RA, old_RB, count_R_add
    volt = GPIO.input(pin)
    if(GPIO_ENC_RA == pin) :
        new_RA = volt
    else :
        new_RB = volt
    if(flag_pin_R == pin) :
        #print "逆回転だぜ！"
        count_R_add *= -1
    elif(old_RA == old_RB) :
        if(old_RA != new_RA) :
            count_R_add = 1
        else :
            count_R_add = -1
    else :
        if(old_RA == new_RA) :
            count_R_add = 1
        else :
            count_R_add = -1
    count_R += count_R_add
    old_RA = new_RA
    old_RB = new_RB
    flag_pin_R = pin
    keisan()


def keisan() :
    global time_old, count_L, count_R, kakusokudo_old, shisei_old, zahyou_x_old, zahyou_y_old, sokudo_old, sokudo_L, sokudo_R
    time_now = time.time()
    if(time_now - time_old > time_interval) :
        time_interval_dt = time_now - time_old
        kakudo_L = (2.0 * math.pi * count_L) / Enc_P
        kakudo_R = (2.0 * math.pi * count_R) / Enc_P
        kakusokudo_L = kakudo_L / Gear / time_interval_dt
        kakusokudo_R = kakudo_R / Gear / time_interval_dt
        sokudo_L = kakusokudo_L * Wheel_W
        sokudo_R = kakusokudo_R * Wheel_W
        sokudo = (sokudo_R + sokudo_L) / 2.0
        kakusokudo = (kakusokudo_R - kakusokudo_L) * Wheel_W / Tread
        shisei = (kakusokudo + kakusokudo_old) * time_interval_dt / 2.0 + shisei_old
        zahyou_x = (sokudo * math.cos(shisei) + sokudo_old * math.cos(shisei_old)) * time_interval_dt / 2.0 + zahyou_x_old
        zahyou_y = (sokudo * math.sin(shisei) + sokudo_old * math.sin(shisei_old)) * time_interval_dt / 2.0 + zahyou_y_old
        #kokokara print
        print "速度  : " + str(sokudo)
        #print "座標x : " + str(zahyou_x)
        #print "座標y : " + str(zahyou_y)
        #print "姿勢  : " + str(shisei / math.pi * 180) + " deg  ||  " +str(shisei) + " rad"
        print "-----"
        file_add = str(zahyou_x) + "\t" + str(zahyou_y) + "\t" + str(shisei) + "\n"
        odometori_file_a.write(file_add)
        #odometori_file_a.close()
        count_L = 0
        count_R = 0
        time_old = time_now
        kakusokudo_old = kakusokudo
        sokudo_old = sokudo
        shisei_old = shisei
        zahyou_x_old = zahyou_x
        zahyou_y_old = zahyou_y


# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_ENC_LA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_ENC_LB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_ENC_RA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_ENC_RB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# event setup
GPIO.add_event_detect(GPIO_ENC_LA, GPIO.BOTH, enc_count_L)
GPIO.add_event_detect(GPIO_ENC_LB, GPIO.BOTH, enc_count_L)
GPIO.add_event_detect(GPIO_ENC_RA, GPIO.BOTH, enc_count_R)
GPIO.add_event_detect(GPIO_ENC_RB, GPIO.BOTH, enc_count_R)
# event wait
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
    odometori_file_a.close()

