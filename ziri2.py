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
run_speed = 2.0   #走行時の移動速度（時速）
Target_x = 0
Target_y = 0
Target_def = 0.1    #目標との容認誤差。
brightness_stop = 800
kakudo = 0

#PWM用
Moter_L1_Pin = 23
Moter_L2_Pin = 24
Moter_R1_Pin = 25
Moter_R2_Pin = 11
PWM_freq = 250.0
PWM_power = 100
speed_MAX = 10.8

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
    #仮想の1m先を計算
    x_vir = math.cos(shisei_now) + x_now
    y_vir = math.sin(shisei_now) + y_now
    print "x_vir : " + str(x_vir)
    print "y_vir : " + str(y_vir)
    #targetにとりあえず1,000かける
    mokuteki_x = Target_x * 1000
    mokuteki_y = Target_y * 1000
    #自分の位置と目標の角度計算
    bunshi = ( (x_vir - x_now) * (mokuteki_y - x_now) - (y_vir - y_now) * (mokuteki_x - x_now) ) 
    #bunshi = x_vir * y_vir + Target_x * Target_y
    bunbo = ( math.sqrt((x_vir - x_now) * (x_vir - x_now) + (y_vir - y_now) * (y_vir - y_now)) * math.sqrt((mokuteki_x - x_now) * (mokuteki_x - x_now) + (mokuteki_y - y_now) * (mokuteki_y - y_now)))
    #bunbo = math.sqrt((x_vir * y_vir) * (x_vir * y_vir)) * math.sqrt((Target_x * Target_y) * (Target_x * Target_y))
    print "分子 : " + str(bunshi)
    print "分母 : " + str(bunbo)
    tau = bunshi / bunbo
    kakudo = math.acos(tau)
    #kakudo = math.asin(tau)
    rotation_run(kakudo)

#log読み取り
def log_read(read_step) :
    global Target_x, Target_y, log_list_step_now
    log_now = log_list[read_step].split('\t')
    Target_x = float(log_now[0])
    Target_y = float(log_now[1])
    print "x : " + str(Target_x) + "\t" + "y : " + str(Target_y)
    print "読み込んだlogの行数 : " + str(log_list_step_now)
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

a_L = 0
a_R = 0

#走行、引数は速度(+x=正転, -x=逆転)、回転方向(1=cw, -1=ccw, 0=straight)
def run_cal(speed, way) :
    #回転方向からホイールの速度を定める
    global a_L, a_R
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
    print Target_speed_L
    print sokudo_Wheel_L
    diff_speed_L_1st = Target_speed_L - sokudo_Wheel_L
    diff_speed_R_1st = Target_speed_R - sokudo_Wheel_R
    diff_speed_L_2nd = diff_speed_L_1st / 5.0
    diff_speed_R_2nd = diff_speed_R_1st / 5.0
    print "速度の引き算 : " + str(diff_speed_L_2nd)
    #速度の決定
    if Target_speed_L == sokudo_Wheel_L :
        speed_L = Target_speed_L
    else :
        speed_L = a_L + diff_speed_L_2nd
    if  Target_speed_R == sokudo_Wheel_R :
        speed_R = Target_speed_R
    else :
        speed_R = a_R + diff_speed_R_2nd
    a_L = speed_L
    a_R = speed_R
    print "現在の速度 : " + str(sokudo_Wheel_L)
    print "指示速度_L : " + str(speed_L)
    print "指示速度_R : " + str(speed_R)
    run_PWM(speed_L, speed_R)

#回転、引数はrad
def rotation_run(Target_kakudo) :
    #+-5degでないときは回す
    kakudo_now = (shisei_old / math.pi) * 180 % 360
    if kakudo_now > Target_kakudo + 5 :
        run_cal(run_speed,1)
    elif kakudo_now < Target_kakudo -5 :
        run_cal(run_speed,-1)
    #+-5degになったらとめる
    else :
        global status_step_now
        run_cal(0,0)
        status_step_now = 2

def close_end() :
    print "おわり\n"
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
    global time_old, count_L, count_R, kakusokudo_old, shisei_old, zahyou_x_old, zahyou_y_old, sokudo_old, sokudo_Wheel_L, sokudo_Wheel_R
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
        sokudo_old = sokudo
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
                run_cal(0,0)
                status_step_now = 0
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
    fileopen.close()

