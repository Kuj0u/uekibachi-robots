# -*- coding: utf-8 -*-

###これ変えてね####
#''のfilenameを適切に変更してください。
#フォーマットは  x座標 (tab) y座標 (tab) 姿勢 (改行)
#正直、姿勢いらないけどね。まとめて取り込んでる。
log_file_name = 'log_odometori.txt'

#走行モード
#1:log追従モード
#2:sensor使用、環境に応じて移動
run_mode = 2

#給水ステーションの座標
water_pos = [2,8]

#######

import RPi.GPIO as GPIO
import time
import math
import datetime
import serial
import socket

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

#自律移動用の変数群
status_step_now = 0
run_speed = 1.0  #走行時の移動速度（時速）
Target_x = 0
Target_y = 0
Target_def = 0.1   #目標との容認誤差。
brightness_stop = 800
diff_kakudo = 0
Target_precision = 0
Target_distance = 0
now_distance = 0
run_way = "none"
flag = 0

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
PWM_power = 100.0
speed_MAX = 10.8
pwm_power_L = 0
PWM_power_R = 0

#PID用的な(笑)泣きそう
#PID_Kp = 0.3
#PID_Ki = 0.4
#PID_Kd = 0.4
PID_Kp = 1
PID_Ki = 1
PID_Kd = 1
PID_I_L = 0
PID_I_R = 0
PID_time_old = time.time()
PID_L = 0
PID_R = 0

#udp
host = "127.0.0.1"
port = 3333
bufsize = 512
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#sensor
light = 0
soil_water = 0
hot = 0
humidity = 0
soil_water_limit = 800
light_limit = 800
light_distance_scope = 0.4 #四角の半径というか、1/2辺の長さ+-でやる
grid_range = [-2, -1, 0, 1, 2]

##########################

#ターゲットまで移動
def move_target(x, y):
    global status_step_now, Target_precison
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
            
            #精密ターゲットが設定されている場合
            if Target_precision == 1 :
                #目標とズレが少ない場合
                if zahyou_def() == 1 :
                    Target_precison = 0
                    status_step_now =0
                #ズレが大きれば
                else :
                    #statusをもどす
                    status_step_now -= 2
            #精密指定なければ
            else :
                status_step_now = 0
        #目標距離まで届いていなければ
        else :
            #走る
            run_cal(run_speed, 0)

#udp
def udp_recv():
    sock.bind((host,port))
    udp_list = sock.recv(bufsize).split("\t")
    return udp_list

#sensorの値
def sensor_info():
    global light, soil_water, hot, humidity
    list_val = udp_recv()
    if (list_val[0] == "start" and list_val[7] == "\n"):
        light = (float(list_val[1]) + float(list_val[2]) + float(list_val[3])) / 3.0
        soil_water = float(list_val[4])
        hot = float(list_val[5])
        humidity = float(list_val[6])

#目標までの距離計算
def target_distance_cal() :
    global Target_distance, now_distance
    Target_distance =  math.sqrt((Target_x - zahyou_x_old) * (Target_x - zahyou_x_old) + (Target_y - zahyou_y_old) * (Target_y - zahyou_y_old))
    now_distance

#座標のズレを評価. 0=まだ離れてる 1=かなり近いね
def zahyou_def() :
    if Target_x > (zahyou_x_old - Target_def) and Target_x < (zahyou_x_old + Target_def) :
        if Target_y > (zahyou_y_old - Target_def) and Target_y < (zahyou_y_old + Target_def) :
            return 1
    return 0


#姿勢の計算
def shisei_cal() :
    #とりあえず、位置と姿勢
    global kakudo, diff_kakudo
    v_x = [0,0]
    v_y = [0,0]
    tan = [0,0]
    shisei_now = shisei_old
    x_now = zahyou_x_old
    y_now = zahyou_y_old
    x_vir = math.cos(shisei_now)
    y_vir = math.sin(shisei_now)
    v_x[0] = x_vir
    v_y[0] = y_vir
    #目標までのベクトル
    x_tar = Target_x - x_now
    y_tar = Target_y - y_now
    #単位ベクトル直すため正規化
    vector_len = math.sqrt((x_tar * x_tar) + (y_tar * y_tar))
    v_x[1] = x_tar / vector_len
    v_y[1] = y_tar / vector_len
    #内積と外積で角度と回転方向を計算
    naiseki = v_x[0] * v_x[1] + v_y[0] * v_y[1]
    gaiseki = v_x[0] * v_y[1] - v_y[0] * v_x[1]
    diff_kakudo = (math.atan2(gaiseki, naiseki) * 180) / math.pi
    ##自分の位置と目標の角度計算
    #tan[0] = (math.atan(y_vir / x_vir) * 180.0) / math.pi
    #tan[1] = (math.atan(y_tar / x_tar) * 180.0) / math.pi
    #i = 0
    #while i<2 :
    #    if  v_x[i] > 0 :
    #        if v_y[i] > 0 :
    #            tan[i] = 180 - tan[i]
    ##        else :
    #            tan[i] = 180 - abs(tan[i] - 45) * 2
    #    i+=1
    #bunshi = (x_vir * y_tar) + (y_vir * x_tar)
    #bunbo = (math.sqrt((x_vir * x_vir ) + (y_vir * y_vir))) * (math.sqrt((x_tar * x_tar) + (y_tar *y_tar)))
    #print "分子 : " + str(bunshi)
    #print "分母 : " + str(bunbo)
    #tau = bunshi / bunbo
    #print "tau : " +  str(tau),
    #kakudo = math.acos(tau)
    #diff_kakudo = tan[1] - tan[0]
    #print "差分 %lf (%lf - %lf) " % (diff_kakudo, tan[1], tan[0])
    #print " tan[0] : " + str(tan[0]) + " tan[1] " + str(tan[1]) + " diff_dig : " + str(kakudo)
    #print "kakudo : " + str(kakudo)
    rotation_run(diff_kakudo)

#target指定
def target_set(x, y):
    global Target_x, Target_y
    Target_x = x
    Target_y = y

#log読み取り
def log_read(read_step) :
    log_list_step_now += 1
    log_now = log_list[read_step].split('\t')
    #target_set(float(log_now[0], float(log_now[1]))
    return log_now

#list内部から目的の値(区切り)のみのlistに変換
def single_list(list_data,list_range, basho):
    i = 0
    single_list_val = range(100)
    for i in range(list_range):
        readline_val = list_data[i].split("\t")
        temp_light = float(readline_val[basho])
        #temp_light = int(temp_light)
        single_list_val[i] = temp_light
    return single_list_val

#lightのみのlistに
def log_to_single_light(loglist):
    global light_list
    light_list = single_list(loglist, 100, 2)

#現在地点がlistmaxに近いかどうか
def light_scope(x,y):
    now_pos = now_pos_02(x,y)
    light_max_xy = light_max_pos(0,now_pos[0], now_pos[1])
    i = 0
    if (light_max_xy[i] - light_distance_scope < now_pos[i] and now_pos[i] <light_max_xy[i] - light_distance_scope) :
        i = 1
        if (light_max_xy[i] - light_distance_scope < now_pos[i] and now_pos[i] <light_max_xy[i] - light_distance_scope) :
            return 1
    return 0

#現在地点を0.2単位で返す
def now_pos_02(x, y):
    now_pos_list = [0, 0]
    x *= 10
    x = int(x) / 1
    x -= x % 2
    x *= 0.1
    y *= 10
    y = int(y) / 1
    y -= y % 2
    y *= 0.1
    now_pos_list[0] = x
    now_pos_list[1] = y
    return now_pos_list


#light_max_xy
def light_max_pos(pass_flag, x, y):
    #maxの値を参照
    if pass_flag == 0:
        light_max_val = max(light_list)
    #当該区域の明るさを0にして別の明るさを求める
    else :
        #一時的なリストを取得
        light_list_temp = light_list
        #今の場所0.2単位
        now_pos_list = now_pos_02(x,y)
        #listように座標と配列番号を整える
        x = now_pos_list[0] * 5
        y = now_pos_list[1] * 5
        x = int(x)
        y = int(y)
        i = 0
        j = 0
        #該当区域(縦横0-9のみ範囲外は無視)
        for i in grid_range:
            if (0 <= x - i and x + i <= 9):
                for j in grid_range:
                    if (0 <= y - j and y + j <= 9):
                        list_num = (int(y) * 10) + int(x)
                        #ゼロ埋め
                        light_list_temp[list_num] = int(0)
        #修正をしたlistでmaxを探す
        print light_list_temp
        light_max_val = max(light_list)
    #maxの場所を探す
    xy_num = light_max_val.index(light_max_val)
    pos_x = int(xy_num) % 10
    pos_y = int(xy_num) / 10
    light_max_pos_xy = [pos_x, pos_y]
    return light_max_pos_xy

#PWM信号変換
def run_PWM(speed_L, speed_R) :
    global pwm_power_L, pwm_power_R
    speed_L = speed_L / speed_MAX * PWM_power
    speed_R = speed_R / speed_MAX * PWM_power
    #上限99
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

def motor_stop() :
    run_PWM(0,0)
    #Moter_L1_PWM.ChangeDutyCycle(100)
    #Moter_L2_PWM.ChangeDutyCycle(100)
    #Moter_R1_PWM.ChangeDutyCycle(100)
    #Moter_R2_PWM.ChangeDutyCycle(100)

#走行、引数は速度(+x=正転, -x=逆転)、回転方向(1=左回転, -1=右回転, 0=straight)
def run_cal(speed, way) :
    #回転方向からホイールの速度を定める
    global PID_time_old, PID_I_L, PID_I_R, PID_L, PID_R, run_way
    if way is 0 : 
        Target_speed_L = speed
        Target_speed_R = speed
        run_way= "直進"
    elif way is -1 :
        Target_speed_L = speed
        Target_speed_R = speed * -1
        run_way= "右回転"
    elif way is 1 : 
        Target_speed_L = speed * -1
        Target_speed_R = speed
        run_way= "左回転"
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
    PID_L = PID_Kp * PID_P_L + PID_Ki * PID_I_L + PID_Kd * PID_D_L
    PID_R = PID_Kp * PID_P_R + PID_Ki * PID_I_R + PID_Kd * PID_D_R
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
    #print "目標速度_L : " + str(Target_speed_L),
    #print "目標速度_R : " + str(Target_speed_R)
    #print "指示速度_L : " + str(PID_L),
    #print "指示速度_R : " + str(PID_R)
    #print "現在速度_L : " + str(sokudo_Wheel_L_t0),
    #print "現在速度_R : " + str(sokudo_Wheel_R_t0)
    PID_time_old = PID_time_now
    run_PWM(PID_L, PID_R)

#回転、引数はrad
def rotation_run(Target_kakudo) :
    #Rad => Dig(わかりやすくするために)
    #Target_kakudo = Target_kakudo * 180.0 / math.pi
    #+-5degでないときは回す
    if Target_kakudo > 5.0 :
        run_cal(0.1,1)
    elif Target_kakudo < -5.0 :
        run_cal(0.1,-1)
    #+-5degになったらとめる
    else :
        motor_stop()

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
    global Target_precision,Target_distance, now_distance, flag
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
            print "目標との距離 : %lf (%lf - %lf)" % (Target_distance - now_distance, Target_distance, now_distance)
            print "進行方向:", run_way
            print "PWM_POWER L:%lf R:%lf" % (pwm_power_L, pwm_power_R)
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
                move_target(float(log_list[0]), float(log_list[1]))
                #目的地に到着してるか
                if move_comp == 1:
                    flag =0
        #demoもーど
        elif run_mode == 2:
            #光のmaxを出す
            if flag == 0:
                log_to_single_light(log_list)
                first_pos = [0, 0]
                first_pos = light_max_pos(0, 0, 0)
                move_target(first_pos[0], first_pos[1])
                flag = 1
            #senser読み込み
            if flag == 1:
                serch_light_lock = 0
                motor_stop()
                #udp_sensor_recv
                sensor_info()
                #設定より値が下回ったら、、、特に水分
                if soil_water < soil_water_limit :
                    flag = 2
                elif light < light_limit :
                    flag = 3
                else :
                    time.sleep(3)
            #水分補給動作
            if flag == 2:
                #ターゲットに移動
                move_target(water_pos[0], water_pos[1])
                #到着したら
                if move_comp == 1:
                    motor_stop()
                    time.sleep(10)
                    flag = 3
            if flag == 3:
                if serch_light_lock == 0:
                    serch_light_lock = 1
                    light_target_new = [0, 0]
                    #現在位置周辺がlightMAX時の座標
                    if light_scope(zahyou_old_x, zahyou_old_y) == 1:
                        light_target_new = light_max_pos(1, zahyou_old_x, zahyou_old_y)
                    #離れたところが明るければ
                    else :
                        light_target_new = light_max_pos(0, zahyou_old_x, zahyou_old_y)
                move_target(light_target_new[0],light_target_new[1])
                if move_comp == 1:
                    serch_light_lock = 0
                    motor_stop()
                    flag = 1
        time.sleep(0.001)

finally :
    print "END"
    motor_stop()
    GPIO.cleanup()
    fileopen.close()

