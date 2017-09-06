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

Target_x = 0
Target_y = 0

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

#odometori file set
date_now = datetime.datetime.today()
fmt_filename = "log_ziritu_" + str(date_now.strftime("%Y-%m-%d_%H:%M:%S")) + ".txt"
print fmt_filename
odometori_file_w = open(fmt_filename, "w")
odometori_file_a = open(fmt_filename, "a")
odometori_file_w.close()

#logの読み込み
#現在は一度に全部読み込み
print "読み込み開始"
fileopen = open(log_file_name,'r')
log_list = fileopen.readlines()
log_list_step = int(len(log_list)) - 1
fileopen.close()
log_list_step_now = 5   #最初はゴチャゴチャしてるから５から
print "読み込み完了"

def move_target(x, y):
    global status_step_now, move_comp
    move_comp = 0
    if status_step_now == 0:
        target_set(x,y)
        status_step_now += 1
    if status_step_now == 1 :
        #shisei_calの中で回転までやってます。
        shisei_cal()
        #shisei_cal(厳密にはrotation_run)のなかでstatus =　次ってやってます。
    if status_step_now == 2 :
        #目標までの距離を計算
        target_distance_cal()
        status_step_now += 1
    if status_step_now == 3 :
        #目標距離まで走ったら
        if Target_distance - now_distance < 0 :
            motor_stop()

        #目標距離まで届いていなければ
        else :
            #走る
            run_cal(run_speed, 0)

def target_set(x,y):
    global Target_x, Traget_y
    Target_x = x
    Target_y = y

def log_read(read_step) :
    global log_list_step_now
    log_list_step_now += 1
    log_now = log_list[read_step].split('\t')
    #target_set(float(log_now[0], float(log_now[1]))
    return log_now

def run_PWM(speed_L, speed_R) :
    global pwm_power_L, pwm_power_R
    speed_L = speed_L / speed_MAX * PWM_power
    speed_R = speed_R / speed_MAX * PWM_power
    #上限99で抑える
    if abs(speed_L) > 100 :
        speed_L = speed_L / abs(speed_L) * 99.0
    if abs(speed_R) > 100 :
        speed_R = speed_R / abs(speed_R) * 99.0
    #pwm_power
    pwm_power_L = speed_L
    pwm_power_R = speed_R
    if speed_L >= 0 :
        Duty_L1 = speed_L
        Duty_L2 = 0
    elif speed_L < 0 :
        Duty_L1 = 0
        Duty_L2 = abs(speed_L)
    if speed_R >= 0 :
        Duty_R1 = speed_R
        Duty_R2 = 0
    elif speed_R < 0 :
        Duty_R1 = 0
        Duty_R2 = abs(speed_R)
    Moter_L1_PWM.ChangeDutyCycle(Duty_L1)
    Moter_L2_PWM.ChangeDutyCycle(Duty_L2)
    Moter_R1_PWM.ChangeDutyCycle(Duty_R1)
    Moter_R2_PWM.ChangeDutyCycle(Duty_R2)
    #Moter_L1_PWM.ChangeDutyCycle(0)
    #Moter_L2_PWM.ChangeDutyCycle(0)
    #Moter_R1_PWM.ChangeDutyCycle(0)
    #Moter_R2_PWM.ChangeDutyCycle(0)

def motor_stop():
    time.sleep(0.001)

def enc_count_L(pin) :
    global count_L, count_R, countdata, count_L_add, count_R_add
    countdata = int(encoderdata.readline())
    count_L_add = (3 & countdata) - 2
    count_R_add = (3 & (data / 100)) - 2
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
    global Target_distance, flag
    print_time_old = time.time()
    while True:
        if time.time() - print_time_old > 0.1 :
            print_time_old = time.time()
            print "現在のステップ : %d" % (status_step_now)
            print "読み込みlog : %d / %d" % (log_list_step_now, log_list_step)
            print "現在座標 x:%lf y:%lf" % (zahyou_x_old, zahyou_y_old)
            print "目標座標 x:%lf y:%lf" % (Target_x, Target_y)
            print "現在姿勢 : %lf" % (shisei_old*180.0/math.pi)
            print "偏差角度 : %lf" % diff_kakudo
            #print "目標との距離 : %lf (%lf - %lf)" % (Target_distance - now_distance, Target_distance, now_distance)
            #print "進行方向:", run_way
            print "PWM_POWER L:%lf R:%lf" % (pwm_power_L, pwm_power_R)
            #print "センサ　現在　light:%lf mizu:%lf" % (light, soil_water)
            #print "センサ　設定　light:%lf mizu:%lf" % (light_limit, soil_water_limit)
            #print "行動:",ima
        #追従モード
        if run_mode == 1:
            #logからターゲット設定
            if flag == 0 :
                #今が最終行でなければ
                if log_list_step_now != log_list_step :
                    target_xy = log_read(log_list_step_now)
                    flag += 1
                #最終行で閉じる
                else :
                    close_end()
            if flag == 1:
                move_target(float(target_xy[0]), float(target_xy[1]))
                #目的地に到着してるか
                if move_comp == 1:
                    flag =0
        #demoもーど
        time.sleep(0.001)

finally :
    print "END"
    motor_stop()
    GPIO.cleanup()
    fileopen.close()
