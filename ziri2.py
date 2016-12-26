# -*- coding: utf-8 -*-

###これ変えてね####
#''のfilenameを適切に変更してください。
#フォーマットは  x座標 (tab) y座標 (tab) 姿勢 (改行)
#正直、姿勢いらないけどね。まとめて取り込んでる。
log_file_name = 'log_odometori.txt'


#######

import RPi.GPIO as GPIO
import time
import math
import datetime
import serial

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

#keisan
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
sokudo_t = 0
sokudo_Wheel_L_t0 = 0
sokudo_Wheel_L_t1 = 0
sokudo_Wheel_L_t2 = 0
sokudo_Wheel_R_t0 = 0
sokudo_Wheel_R_t1 = 0
sokudo_Wheel_R_t2 = 0

#odometori file set
#date_now = datetime.datetime.today()
#fmt_filename_ = "log_ziritu_light_" + str(date_now.strftime("%Y-%m-%d_%H:%M:%S")) + ".txt"
#print fmt_filename
#odometori_file_w = open(fmt_filename, "w")
#odometori_file_a = open(fmt_filename, "a")
#odometori_file_w.close()

#logの読み込み
#現在は一度に全部読み込み
print "読み込み開始"
fileopen = open(log_file_name,'r')
log_list = fileopen.readlines()
log_list_step = int(len(log_list)) - 1
fileopen.close()
log_list_step_now = 0
print "読み込み完了"

#自律移動用の変数群
status_step_now = 0
run_speed = 1.0  #走行時の移動速度（時速）
Target_x = 0
Target_y = 0
Target_def = 0.1    #目標との容認誤差。
brightness_stop = 800
kakudo = 0

#PWM用
#Moter_L1_Pin = 23
#Moter_L2_Pin = 24
#Moter_R1_Pin = 25
#Moter_R2_Pin = 11
Moter_R1_Pin = 23
Moter_R2_Pin = 24
Moter_L1_Pin = 25
Moter_L2_Pin = 11
PWM_freq = 250.0
PWM_power = 100
speed_MAX = 10.8

#PID用的な(笑)泣きそう
PID_Kp = 0.7
PID_Ki = 0.7
PID_Kd = 0.5
PID_I_L = 0
PID_I_R = 0
PID_time_old = 0
PID_L = 0
PID_R = 0
##########################33

#座標のズレを評価. 0=まだ離れてる 1=かなり近いね
def zahyou_def() :
    if Target_x > (zahyou_x_old - Target_def) and Target_x < (zahyou_x_old + Target_def) :
        if Target_y > (zahyou_y_old - Target_def) and Target_y < (zahyou_y_old + Target_def) :
            return 1
    return 0


#姿勢の計算
def shisei_cal() :
    #とりあえず、位置と姿勢
    global kakudo
    shisei_now = shisei_old
    x_now = zahyou_x_old
    y_now = zahyou_y_old
    #仮想の1m先(仮想ベクトル)を計算
    x_vir = math.cos(shisei_now)
    y_vir = math.sin(shisei_now)
    #目標までのベクトル
    x_tar = Target_x - x_now
    y_tar = Target_y - y_now
    #単位ベクトル直すため長さ求める
    vector_len = x_tar + y_tar
    seikika = 1.0 / vector_len
    print "正規化 : " + str(seikika)
    #正規化
    x_tar = x_tar * seikika
    y_tar = y_tar * seikika
    #自分の位置と目標の角度計算
    bunshi = (x_vir * y_tar) - (y_vir * x_tar)
    bunbo = (math.sqrt((x_vir * x_vir ) + (y_vir * y_vir))) * (math.sqrt((x_tar * x_tar) + (y_tar *y_tar)))
    print "分子 : " + str(bunshi)
    print "分母 : " + str(bunbo)
    tau = bunshi / bunbo
    print "tau : " +  str(tau)
    kakudo = math.acos(tau)
    rotation_run(kakudo)

#log読み取り
def log_read(read_step) :
    global Target_x, Target_y, log_list_step_now
    log_now = log_list[read_step].split('\t')
    Target_x = float(log_now[0])
    Target_y = float(log_now[1])
    print "読み込んだlogの行数 : " + str(log_list_step_now)
    print "読み込んだ座標 : ",
    print "x : " + str(Target_x) + "\t" + "y : " + str(Target_y)
    log_list_step_now += 1

#PWM信号変換
def run_PWM(speed_L, speed_R) :
    speed_L = speed_L / speed_MAX * PWM_power
    speed_R = speed_R / speed_MAX * PWM_power
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

def motor_stop() :
    Moter_L1_PWM.ChangeDutyCycle(100)
    Moter_L2_PWM.ChangeDutyCycle(100)
    Moter_R1_PWM.ChangeDutyCycle(100)
    Moter_R2_PWM.ChangeDutyCycle(100)

#走行、引数は速度(+x=正転, -x=逆転)、回転方向(1=cw, -1=ccw, 0=straight)
def run_cal(speed, way) :
    #回転方向からホイールの速度を定める
    global PID_time_old, PID_I_L, PID_I_R, PID_L, PID_R
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
    #PID制御でやる。パラメータ適宜調整
    PID_time_now = time.time()
    PID_dt = PID_time_now - PID_time_old
    PID_P_L = Target_speed_L - sokudo_Wheel_L_t0
    PID_P_R = Target_speed_R - sokudo_Wheel_R_t0
    PID_I_L = (Target_speed_L - sokudo_Wheel_L_t0) * PID_dt + PID_I_L
    PID_I_R = (Target_speed_R - sokudo_Wheel_R_t0) * PID_dt + PID_I_R
    PID_D_L = (Target_speed_L - sokudo_Wheel_L_t0) - (sokudo_Wheel_L_t1 - sokudo_Wheel_L_t2)
    PID_D_R = (Target_speed_R - sokudo_Wheel_R_t0) - (sokudo_Wheel_R_t1 - sokudo_Wheel_R_t2)
    #操作量
    add_L = PID_Kp * PID_P_L + PID_Ki * PID_I_L + PID_Kd * PID_D_L
    add_R = PID_Kp * PID_P_R + PID_Ki * PID_I_R + PID_Kd * PID_D_R
    #目標速度に操作量を追加
    PID_L = PID_L + add_L
    PID_R = PID_R + add_R
    ##目標速度までの差分計算 P制御
    #diff_speed_L_1st = Target_speed_L - sokudo_Wheel_L
    #diff_speed_R_1st = Target_speed_R - sokudo_Wheel_R
    #diff_speed_L_2nd = diff_speed_L_1st / 5.0
    #diff_speed_R_2nd = diff_speed_R_1st / 5.0
    #print "速度の引き算 : " + str(diff_speed_L_2nd)
    ##速度の決定
    #if Target_speed_L == sokudo_Wheel_L :
    #    speed_L = Target_speed_L
    #else :
    #    speed_L = add_L + diff_speed_L_2nd
    #if  Target_speed_R == sokudo_Wheel_R :
    #    speed_R = Target_speed_R
    #else :
    #    speed_R = add_R + diff_speed_R_2nd
    #add_L = speed_L
    #add_R = speed_R
    #print "現在の速度_L : " + str(sokudo_Wheel_L)
    print "目標速度_L : " + str(Target_speed_L),
    print "目標速度_R : " + str(Target_speed_R)
    print "指示速度_L : " + str(PID_L),
    print "指示速度_R : " + str(PID_R)
    print "現在速度_L : " + str(sokudo_Wheel_L_t0),
    print "現在速度_R : " + str(sokudo_Wheel_R_t0)
    PID_time_old = PID_time_now
    run_PWM(PID_L, PID_R)

#回転、引数はrad
def rotation_run(Target_kakudo) :
    #Rad => Dig(わかりやすくするために)
    Target_kakudo = Target_kakudo * 180.0 / math.pi
    #+-5degでないときは回す
    if Target_kakudo > Target_kakudo + 5.0 :
        run_cal(run_speed,1)
    elif Target_kakudo < Target_kakudo - 5.0 :
        run_cal(run_speed,-1)
    #+-5degになったらとめる
    else :
        global status_step_now
        motor_stop()
        status_step_now = 2

def close_end() :
    print "おわり\n"
    motor_stop()
    GPIO.cleanup()
    #fileopen.close()   #既に閉じてるから大丈夫
    exit()


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
    global time_old, count_L, count_R, kakusokudo_old, shisei_old, zahyou_x_old, zahyou_y_old, sokudo_old, sokudo_Wheel_L, sokudo_Wheel_R, sokudo_Wheel_L_t0, sokudo_Wheel_L_t1, sokudo_Wheel_L_t2, sokudo_Wheel_R_t0, sokudo_Wheel_R_t1, sokudo_Wheel_R_t2
    time_now = time.time()
    if(time_now - time_old > time_interval) :
        time_interval_dt = time_now - time_old
        kakudo_L = (2.0 * math.pi * count_L) / Enc_P
        kakudo_R = (2.0 * math.pi * count_R) / Enc_P
        kakusokudo_L = kakudo_L / Gear / time_interval_dt
        kakusokudo_R = kakudo_R / Gear / time_interval_dt
        sokudo_Wheel_L = kakusokudo_L * Wheel_W / Tread
        sokudo_Wheel_R = kakusokudo_R * Wheel_W / Tread
        sokudo = (sokudo_Wheel_R + sokudo_Wheel_L) / 2.0
        kakusokudo = (kakusokudo_R - kakusokudo_L) * Wheel_W / Tread
        shisei = (kakusokudo + kakusokudo_old) * time_interval_dt / 2.0 + shisei_old
        zahyou_x = (sokudo * math.cos(shisei) + sokudo_old * math.cos(shisei_old)) * time_interval_dt / 2.0 + zahyou_x_old
        zahyou_y = (sokudo * math.sin(shisei) + sokudo_old * math.sin(shisei_old)) * time_interval_dt / 2.0 + zahyou_y_old
        #file_add = str(zahyou_x) + "\t" + str(zahyou_y) + "\t" + str(shisei) + "\n"
        #odometori_file_a.write(file_add)
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
        shisei_old = shisei
        zahyou_x_old = zahyou_x
        zahyou_y_old = zahyou_y
 

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
    global status_step_now
    while True:
        global status_step_now
        print "現在のステップ : " + str(status_step_now)
        if status_step_now == 0 :
            #いろいろ読み込み
            #USBシリアル受け取り.繋いだらコメントアウト解除
            #ser = serial.Serial('/dev/ttyACM0', 9600)
            #brightness_now = int(ser.readline())
            #しきい値よりも明るければ止める
            #if brightness_now > brightness_stop :
            #    print "あかる！！\n"
            #    close_end()
            #リストが終わりでなければ次を読み込む
            print "log_step : " + str(log_list_step)
            print "log_step_now : " + str(log_list_step_now)
            if log_list_step_now != log_list_step :
                log_read(log_list_step_now)
                status_step_now = 1
            #それ以外は終了
            else :
                close_end()
        if status_step_now == 1 :
            #姿勢を整える
            #shisei_calの中で回転までやってます。
            shisei_cal()
            #shisei_cal(厳密にはrotation_run)のなかでstatus =　次ってやってます。
        if status_step_now == 2 :
            #走行します。
            #目標地点でなければ走り続ける
            if zahyou_def() != 1 :
                run_cal(run_speed,0)
            #目標地点なら止まってまた初めからやる
            else :
                motor_stop()
                status_step_now = 0
        time.sleep(1)

#except KeyboardInterrupt:
except :
    motor_stop()
    GPIO.cleanup()
    fileopen.close()

