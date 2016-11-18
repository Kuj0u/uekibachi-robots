# -*- coding: utf-8 -*-

###これ変えてね####
#''のfilenameを適切に変更してください。
#フォーマットは  x座標 (tab) y座標 (tab) 姿勢 (改行)
log_file_name = ''


#######

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
time_interval = 0.1
time_interval_dt = 0
kakusokudo_old = 0
shisei_old = 0
zahyou_x_old = 0
zahyou_y_old = 0
sokudo_old = 0
sokudo_Wheel_L = 0
sokudo_Wheel_R = 0

#odometori file set
#date_now = datetime.datetime.today()
#fmt_filename_ = "log_ziritu_light_" + str(date_now.strftime("%Y-%m-%d_%H:%M:%S")) + ".txt"
#print fmt_filename
#odometori_file_w = open(fmt_filename, "w")
#odometori_file_a = open(fmt_filename, "a")
#odometori_file_w.close()

#logの読み込み
#現在は一度に全部読み込み
fileopen = open(log_file_name,'r')
log_list = fileopen.readlines()
log_list_step = len(fileopen.readlines())
log_list = log_list
log_list_step_now = 0

#自律移動用の変数群
status_step_now = 0
run_speed = 2.0   #走行時の移動速度（時速）
Target_x = 0
Target_y = 0

#PWM用
Moter_L1_Pin = 23
Moter_L2_Pin = 24
Moter_R2_Pin = 25
Moter_Rb_Pin = 11
PWM_frea = 250.0
PWM_power = 100
speed_MAX = 10.0

##########################33

def shisei_cal() :
    #とりあえず、位置と姿勢
    shisei_now = shisei_old
    x_now = zahyou_x_old
    y_now = zahyou_y_old
    #仮想の1m先を計算
    x_vir = math.cos(x_now)
    y_vir = math.sin(y_now)
    #自分の位置と目標の角度計算
    tau = ( (x_vir - x_now) * (Target_y - x_now) - (y_vir - y_now) * (Target_x - x_now) ) / ( math.sqrt((x_vir - x_now) * (x_vir - x_now) + (y_vir - y_now) * (y_vir - y_now)) * math.sqrt((Target_x - x_now) * (Target_x - x_now) + (Target_y - y_now) * (Target_y - y_now)))
    sabun_kakudo = math.asin(tau)
    rotation_run(sabun_kakudo)

#log読み取り
def log_read(read_step) :
    global Target_x, Target_y, log_list_step_now
    log_now = log_list[read_step].split('\t')
    Target_x = log_now[0]
    Target_y = log_now[1]

#PWM信号変換
def run_PWM(speed_L, speed_R) :
    speed_L = speed_L / speed_MAX * PWM_power
    speed_R = speed_R / speed_MAX * PWM_power
    if speed_L >= 0 :
        Duty_L1 = 0
        Duty_L2 = speed_L
    elif speed_L < 0 :
        Duty_L1 = abs(speed_L)
        Duty_L2 = 0
    if speed_R >= 0 :
        Duty_R1 = 0
        Duty_R2 = speed_R
    elif speed_R < 0 :
        Duty_R1 = abs(speed_R)
        Duty_R2 = 0
    Moter_L1_PWM.ChangeDutyCycle(Duty_L1)
    Moter_L2_PWM.ChangeDutyCycle(Duty_L2)
    Moter_R1_PWM.ChangeDutyCycle(Duty_R1)
    Moter_R2_PWM.ChangeDutyCycle(Duty_R2)


#走行、引数は速度(+x=正転, -x=逆転)、回転方向(1=cw, -1=ccw, 0=straight)
def run_cal(speed, way) :
    #回転方向からホイールの速度を定める
    if way is 0 : 
        Target_speed_L = speed
        Target_speed_R = speed
    elif way is 1 :
        Target_speed_L = speed
        Target_speed_R = speed * -1
    elif way is 2 : 
        Target_speed_L = speed * -1
        Target_speed_R = speed
    #目標速度までの差分計算
    diff_speed_L_1st = Target_speed_L - sokudo_Wheel_L
    diff_speed_R_1st = Target_speed_R - sokudo_Wheel_L
    diff_speed_L_2nd = diff_speed_L_1st / 50.0
    diff_speed_R_2nd = diff_speed_R_1st / 50.0
    #速度の決定
    if sokudo_Wheel_L is Target_speed_L:
        speed_L = Target_speed_L
    else :
        speed_L = sokudo_Wheel_L + diff_speed_L_2nd
    if sokudo_Wheel_R is Target_speed_R :
        speed_R = Target_speed_R
    else :
        speed_R = sokudo_Wheel_R + diff_speed_R_2nd
    run_PWM(speed_L, speed_R)

#回転、引数はrad
def rotation_run(Target_kakudo) :
    #+-5degでないときは回す
    kakudo_now = (shisei_old % 360) / 180 * math.pi
    if kakudo_now > Target_kakudo + 5 :
        run_cal(1,1)
    elif kakudo_now < Target_kakudo -5 :
        run_cal(1,-1)
    #+-5degになったらとめる
    else :
        global status_step_now
        run_cal(0,0)
        status_step_now = 2


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
        sokudo_Wheel_L = kakusokudo_L * Wheel_W / Tread
        sokudo_Wheel_R = kakusokudo_R * Wheel_W / Tread
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
        #現在の状態 status_nowで作業内容に移る
        #現在位置と目標位置が一致しているか
            #次の位置と角度を用意
        #statusの比較
        global status_step_now
        if(stasus_step_now == 0) :
            #目標の読み込み
            #最終行でなければ読み込み
            if log_list_step_now != log_list_step :
                log_read(log_list_step_now)
                status_step_now = 1
            else
                print "おわり\n"
                GPIO.cleanup()
                fileopen.close()
                exit()
        if(status_step_now == 1) :
            #姿勢を整える
            #shisei_calの中で回転までやってます。
            shisei_cal()
            #shisei_call(厳密にはrotation_run)のなかでstatus =　次ってやってます。
        if(status_step_now == 2) :
            #走行します。
            if run_der() != 1 :
                run_cal(1,0)
            else :
                run_cal(0,0)
                status_step_now = 0
        if(status_step_now == 3) :
            #走行ステップ
            #run()にと速度を送る。
            #if 目標位置 == 現在位置
                #status = 0
                #目標設定に戻る



# GPIO setup
GPIO.setmode(GPIO.BCM)

#setup encoder
GPIO.setup(GPIO_ENC_LA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_ENC_LB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_ENC_RA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_ENC_RB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

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
    fileopen.close()

