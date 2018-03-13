#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import serial
import time
import math
import RPi.GPIO as GPIO
from module import Astar, mapview, move, sensor_read, data_write, data_read, data_show, node_create
from module import ClientSetting, SendData, ReciveData, closing, PosRead
import math

# Arduinoとのシリアル通信
arduino = serial.Serial('/dev/ttyACM0', 230400, timeout=1)

# オドメトリファイル
f = open("odometry_pid.txt", "w")

# 定数群
PLS_PER_R = 200.0         # ロータリーエンコーダの1回転分パルス数 [パルス/回転]
MUL       = 4.0           # 逓倍数
GEAR      = 5.0           # タイヤを1とした際のロータリエンコーダの回転比率
WHEEL     = 100.0 / 1000.0  # タイヤ半径 [m]
TREAD     = 366.6 / 1000.0  # 車輪幅 [m]
ENC_R = PLS_PER_R * MUL * GEAR

# グローバル変数

dt = float(0.1) 	# Arduino側が100ms毎にカウント値やってるんで一応このまま
runvalue = [0, 0]	# mainloop()内でなんかエラー吐かれるからグローバルにしときます
viewtime = time.time()	# 表示速度クソ早いんで一定周期に表示するように
oldtime = 0
viewflag = 0

#オドメトリ
x = 0
y = 0
th_old = 0
t_old = 0
v_old = 0
omega_th = 0
omega_th_old = 0
cc = 0
countL_dt = 0
countR_dt = 0
countL_old = 0
countR_old = 0

# 移動判定用
nowpos			= [0, 0]	# 現在位置 [mm]
oldpos			= [0, 0]	# 前回の位置 [mm]
tplist			= list()	# 移動経路リスト
firstp			= [0, 0]	# 初期位置
tp				= [0, 0]	# target position [mm]
tr				= 0.0 		# target radius [rad]
diffrad_run		= 20.0 / 180	# 許容誤差角度 [deg]
diffrad_rotate	= 10.0 / 180
diffdist		= 0.10		# 許容誤差距離 [mm]
move_event		= 0
move_event_old		= 6

# PD制御用の各変数
PDwheelv_L 		= 0.0		# 左：今回偏差(角度)
PDwheelv_Lt0	= 0.0		# 左：前回偏差
PDwheelv_Lt1	= 0.0		# 左：前々回偏差
PDwheelv_R 		= 0.0		# 右：今回偏差(角度)
PDwheelv_Rt0	= 0.0		# 右：前回偏差
PDwheelv_Rt1	= 0.0		# 右：前々回偏差
PD_Kp		 	= 100.0		# P定数　制御量おかしかったらここ調整
PD_Kd		 	= 20.0		# D定数　同じ
PD_PL	 		= 0			# 左：P
PD_PR	 		= 0			# 右：P
PD_DL	 		= 0			# 左：D
PD_DR	 		= 0			# 右：D
PD_Lold			= 0			# 左：前回の操作量
PD_Rold			= 0			# 右：前回の操作量

# PID用の各変数
PID_Kp 		  = 17.0			# P定数　制御量おかしかったらここ調整
PID_Ki 		  = 0.001			# I定数　同じ
PID_Kd		  = 35.0			# D定数　同じ
PID_PL 		  = 0.0			# 左：P
PID_IL 		  = 0.0			# 左：I
PID_DL		  = 0.0			# 左：D
PID_PR 		  = 0.0			# 右：P
PID_IR 		  = 0.0			# 右：I
PID_DR 		  = 0.0			# 右：D
PIDwheelv_L   = 0.0			# 左：今回偏差(距離)
PIDwheelv_Lt0 = 0.0			# 左：前回偏差
PIDwheelv_Lt1 = 0.0			# 左：前々回偏差
PIDwheelv_R   = 0.0			# 右：今回偏差(距離)
PIDwheelv_Rt0 = 0.0			# 右：前回偏差
PIDwheelv_Rt1 = 0.0			# 右：前々回偏差
PID_Lold 	  = 0.0
PID_Rold	  = 0.0

# PWM用
Moter_L1_Pin = 23
Moter_L2_Pin = 24
Moter_R1_Pin = 25
Moter_R2_Pin = 5
PWM_freq = 250.0
minpwm = 90.0
Duty = 75.0
rad_ad_L = 0
rad_ad_R = 0
om_ad_L = 0
om_ad_R = 0
m = 1
Duty_L1 = 0
Duty_L2 = 0
Duty_R1 = 0
Duty_R2 = 0
Duty_L1_old = 0
Duty_R1_old = 0

# GPIO setup
GPIO.setmode(GPIO.BCM)
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

# Arduinoシリアル受信
# 戻り値: (Arduino時間[ms], 左カウント数, 右カウント数)
def read_counter_from_arduino():
	global cc
	while True:
		recv = arduino.readline() # arduinoからの受信した文字列
	        # スペースで分離
		recv_split = recv.strip().split(" ") # (時間, 左カウント数, 右カウント数)
		#print "serial:", recv_split, len(recv_split)
		if len(recv_split) == 3: # 文字数チェック
			deltime   = int(recv_split[0])
			counter_l = int(recv_split[1])
			counter_r = int(recv_split[2])
			#cc += counter_l
			#print cc
            		return (deltime, counter_l, counter_r)


# オドメトリ計算
# 引数: 時間t[s], カウント数(2要素)
# 戻り値: (車体x[m], 車体y[m], 車体角度[rad], 並進速度v[m/h], 車輪速度omega_l, omega_r)
def calc_odometry(t, count):
    global t_old, v_old, omega_th, omega_th_old, th_old, x, y, countL_dt, countR_dt, countL_old, countR_old, dt
    global ENC_R, firstp, nowpos, viewflag

	# 車輪のカウントdt
    countL_dt = float(count[0]) #- float(countL_old)
    countR_dt = float(count[1]) #- float(countR_old)

    # タイヤ加速度
    omega_l = (2.0 * math.pi * countL_dt) / (GEAR * dt * ENC_R)
    omega_r = (2.0 * math.pi * countR_dt) / (GEAR * dt * ENC_R)

    # 並進速度
    v = (omega_l + omega_r) * WHEEL / 2.0

    # 車体回転角速度
    omega_th = (omega_r - omega_l) * WHEEL / TREAD

    # 台形積分
    th = th_old + (omega_th + omega_th_old) * dt / 2.0 * 5.0
    x = x + (v * math.cos(th) + v_old * math.cos(th_old)) * dt * 5.0 / 2.0
    y = y + (v * math.sin(th) + v_old * math.sin(th_old)) * dt * 5.0 / 2.0

    # nowposにぶち込み
    nowpos = [x+firstp[0], y+firstp[1]]
    # 更新
    t_old = t
    v_old = v
    omega_th_old = omega_th
    th_old = th
    countL_old = float(count[0])
    countR_old = float(count[1])
    # print ("countL:%d counrR:%d dt:%03.3f" %(int(count[0]), int(count[1]), dt))
    # print ("v:%03.3f") %(v * 360)
    if viewflag == 1:
	    print("omega_l = %03.3f omega_r = %03.3f") % (omega_l, omega_r)
    return (x + firstp[0], y + firstp[1], th, v * 60, omega_l, omega_r)



# 移動判定
# 移動の流れとしては　目標位置確認→回転→直進→停止(オーバーしたらバック)
# 処理上の関係で「目標地点が近すぎると回転しない」（ここ検討の余地あり）
# 引数：現在地点np[x,y] 現在角度[rad], 並進速度v[m/h]
# 戻り値：イベントフラグ1~4, 目標までの直線距離[mm], 目標までの角度[rad]
def move_judgement(np, now_rad, v):
	global tp, diffrad, diffdist, tr, move_event, move_event_old, viewflag

	# 目標までの直線距離 これが現在偏差距離になります
	target_dist = math.sqrt( (tp[0]-np[0]) * (tp[0]-np[0]) + (tp[1] - np[1]) * (tp[1] - np[1]) )

	# 目標までの角度 同様に現在偏差角度
	target_rad = math.atan2(tp[1] - np[1], tp[0] - np[0]) - now_rad

	# 制動距離 ブレーキかけて止まるまでの距離　時速^2 / (254 * 摩擦係数)
	seidou = (v * v) / (254 * 0.7)

	# イベントフラグ設定 1:反時計回り(左折) 2:時計回り(右折) 3:直進 4:後進 5:制動ストップ 6:完全停止

	# 制動距離が目標までの距離より大きくなったら停止させる
	if seidou > abs(target_dist):
		move_event = 5

	# 目標までの距離が許容範囲内であれば停止
	#if target_dist < diffdist:
	#	move_event = 6

	# バック・直進。目標までの距離が許容距離範囲外であれば実行
	elif abs(target_rad) > diffrad_rotate:

	# イキスギィ！てたらバック
		if np[0] - tp[0] < 0 and target_dist < diffdist:
			#print "x4"
			move_event = 4

	# イキスギィ！てたらバック
		elif  np[1] - tp[1] < 0 and target_dist < diffdist:
			move_event = 4
			#print "y4"

		else:
			if target_rad < 0:
				move_event = 1
			elif target_rad > 0:
				move_event = 2


	elif target_dist > diffdist:
		move_event = 3
	elif target_dist < diffdist:
		move_event = 6

	# 前回の状態を記憶
	move_event_old = move_event

	move_str = 'WTF'
	# Debug
	if move_event == 1:
		move_str = '右折'
	elif move_event == 2:
		move_str = '左折'
	elif move_event == 3:
		move_str = '直進'
	elif move_event == 4:
		move_str = 'バック'
	elif move_event == 5:
		move_str = '制動停止'
	elif move_event == 6:
		move_str = '完全停止'

	if viewflag == 1:
		#print "targedist:%03.3f, targetrad:%03.3f, move_event:%d, 制動距離:%03.3f" %(target_dist, target_rad * 180 / math.pi, move_event, seidou)
		print "目標距離:%03.3f, 目標角度:%03.3f, 動作flag:%s"%(target_dist, target_rad*180/math.pi, move_str)
		print "tp = [%03.3f %03.3f], np = [%03.3f %03.3f]" %(tp[0], tp[1], np[0], np[1])
	return (move_event, target_dist, target_rad)



# 回転(PD制御)
# 引数：イベントフラグevent, 目標までの角度target_r[rad], 現在角度 nowrad[rad]
# 戻り値：制御量　左車輪PD_L, 右車輪PD_R
def calc_rotation(event, target_r, nowrad) :
	global PD_Lold, PD_Rold, PD_Kp, PD_Kd, PD_PL, PD_PR, PD_DL, PD_DR
	global PDwheelv_L, PDwheelv_Lt0, PDwheelv_Lt1, PDwheelv_R, PDwheelv_Rt0, PDwheelv_Rt1

	target_r = target_r - nowrad
	# 反時計回り
	if event == 1 :
		PDwheelv_L = abs(target_r)
		#PDwheelv_R = abs(target_r) * -1

	# 時計回り
	elif event == 2 :
		#PDwheelv_L = abs(target_r) * -1
		PDwheelv_R = abs(target_r)

	# 各車輪P,D値計算
	PD_PL = PDwheelv_L - PDwheelv_Lt0				# 左：今回偏差 - 前回偏差
	PD_PR = PDwheelv_R - PDwheelv_Rt0				# 右：今回偏差 - 前回偏差
	PD_DL = (PDwheelv_L - PDwheelv_Lt0) - (PDwheelv_Lt0 - PDwheelv_Lt1)	# 左：P - (前回偏差 - 前々回偏差)
	PD_DR = (PDwheelv_R - PDwheelv_Rt0) - (PDwheelv_Rt0 - PDwheelv_Rt1)	# 右：P - (前回偏差 - 前々回偏差)

	# 回転！
	PD_L = PD_Lold + (PD_Kp * PD_PL + PD_Kd * PD_DL)	# 左：前回の操作量 + (PD_Kp * P + PD_Kd * D)
	PD_R = PD_Rold + (PD_Kp * PD_PR + PD_Kd * PD_DR)	# 右：前回の操作量 + (PD_Kp * P + PD_Kd * D)

	# 更新
	PDwheelv_Lt0 = PDwheelv_L
	PDwheelv_Lt1 = PDwheelv_Lt0
	PDwheelv_Rt0 = PDwheelv_R
	PDwheelv_Rt1 = PDwheelv_Rt0
	PD_Lold = PD_L
	PD_Rold = PD_R

	# Debug
	#print "%03.3f, %03.3f" %(PD_L, PD_R)

	# return (PD_L, PD_R)
	return (80, 80)


# 直進制御(PID制御)
# 引数：目標までの距離 targert_d [mm], 移動判定 event(3:直進 4:後進)
# 戻り値：制御量　左車輪PID_L, 右車輪PID_R
def calc_straight(target_d, event) :
	global dt, PID_Lold, PID_Rold, PID_Kp, PID_Kd, PID_PL, PID_PR, PID_IL, PID_IR, PID_DL, PID_DR
	global PIDwheelv_L, PIDwheelv_Lt0, PIDwheelv_Lt1, PIDwheelv_R, PIDwheelv_Rt0, PIDwheelv_Rt1

	# 引数ぶっこみ
	PIDwheelv_L = target_d
	PIDwheelv_R = target_d

	# 各車輪P,I,D値計算
	PID_PL = PIDwheelv_L - PIDwheelv_Lt0					# 左P：今回偏差 - 前回偏差
	PID_PR = PIDwheelv_R - PIDwheelv_Rt0					# 右P：今回偏差 - 前回偏差
	PID_IL = PIDwheelv_L 									# 左I：今回偏差
	PID_IR = PIDwheelv_R 									# 右I：今回偏差
	PID_DL = PID_PL - (PIDwheelv_Lt0 - PIDwheelv_Lt1)		# 左D：P - (前回偏差 - 前々回偏差)
	PID_DR = PID_PR - (PIDwheelv_Rt0 - PIDwheelv_Rt1)		# 右D：P - (前回偏差 - 前々回偏差)

	# 直進！
	PID_L = PID_Lold + (PID_Kp * PID_PL + PID_Ki * PID_IL + PID_Kd * PID_DL)	# 左：前回の操作量 + (PD_Kp * P + PD_Kd * D)
	PID_R = PID_Rold + (PID_Kp * PID_PR + PID_Ki * PID_IR + PID_Kd * PID_DR)	# 右：前回の操作量 + 今回操作量差分

	# 更新
	PIDwheelv_Lt0 = PIDwheelv_L
	PIDwheelv_Lt1 = PIDwheelv_Lt0
	PIDwheelv_Rt0 = PIDwheelv_R
	PIDwheelv_Rt1 = PIDwheelv_Rt0
	PID_Lold = PID_L
	PID_Rold = PID_R

	# 後進なら負の値に
	if event == 4:
		PID_L *= -1
		PID_R *= -1

	# Debug
	#print "%03.3f, %03.3f" %(PID_L, PID_R)

	return (PID_L, PID_R)



# PWM出力
# 戻り値：左右のduty比
def run_pwm(speedL, speedR, event, t_rad, om_r, om_l):
	# global Duty,Duty_L1, Duty_L2, Duty_R1, Duty_R2, Duty_L1_old, Duty_R1_old

	global minpwm, rad_ad_L, rad_ad_R, om_ad_L, om_ad_R, m, viewflag

	k = 1

	# 0で割ると怒られるのでほぼ0にする
	if speedL == 0:
		speedL = 0.0001
	if speedR == 0:
		speedR = 0.0001

	# 直進・バック時の方向補正
	if event == 3 or event == 4:
		# 角度調整
		if float(t_rad) < 0:
			rad_ad_L = abs(t_rad) * k * -1
			rad_ad_R = 0

		if float(t_rad) > 0:
			rad_ad_L = 0
			rad_ad_R = abs(t_rad) * k * -1

		# 車輪の速さが違う時も補正
		if abs(abs(om_r) - abs(om_l)) > 0.3:
			if float(om_r) > float(om_l):
				om_ad_R = abs(om_r) * m * -1
				om_ad_L = 0
			if float(om_r) < float(om_l):
				om_ad_L = abs(om_l) *  m * -1
				om_ad_R = 0

		speedL += om_ad_L
		speedR += om_ad_R

	else:
		speedL += 0
		speedR += 0


	# 符号が逆になることがあるので正しい符号にしてやる
	if event == 1:
		speedL = abs(speedL)
		speedR = abs(speedR) * -1

	if event == 2:
		speedL = abs(speedL) * -1
		speedR = abs(speedR)

	if event == 3:
		speedL = abs(speedL)
		speedR = abs(speedR)

	if event == 4:
		speedL = abs(speedL) * -1
		speedR = abs(speedR) * -1

	#MAX < 99
	if abs(speedL) > 99:
		speedL = speedL / abs(speedL) * 99.0
		if speedL < 0:
			speedL -= om_ad_L 
		elif speedL > 0:
			speedL += om_ad_L 
	if abs(speedR) > 99:
		speedR = speedR / abs(speedR) * 99.0
		if speedR < 0:
			speedR -= om_ad_R
		elif speedR > 0:
			speedR += om_ad_R

	# MIN > Minpwm
	if abs(speedL) < minpwm :
		speedL = (speedL / abs(speedL) * minpwm) - 7.5
	if abs(speedR) < minpwm :
		speedR = speedR / abs(speedR) * minpwm



	# ぶち込む
	if speedL >= 0:
		Duty_L1 = speedL
		Duty_L2 = 0
	elif speedL < 0:
		Duty_L1 = 0
		Duty_L2 = abs(speedL)

	if speedR >= 0:
		Duty_R1 = speedR
		Duty_R2 = 0
	elif speedR < 0:
		Duty_R1 = 0
		Duty_R2 = abs(speedR)

	# 回転
	if event == 1:
		Duty_L1 = abs(speedL)
		Duty_L2 = 0
		Duty_R1 = 00
		Duty_R2 = abs(speedR)

	if event == 2:
		Duty_L1 = 00
		Duty_L2 = abs(speedL)
		Duty_R1 = abs(speedR)
		Duty_R2 = 0


	# 制動停止
	if event == 5 :
		Duty_L1 = abs(speedL)
		Duty_L2 = abs(speedL)
		Duty_R1 = abs(speedR)
		Duty_R2 = abs(speedR)

	# 完全停止
	if event == 6 :
		Duty_L1 = 100
		Duty_L2 = 100
		Duty_R1 = 100
		Duty_R2 = 100


	# Debug
	#print "speed %03.3f %03.3f" %(speedL, speedR)
	#print "%d, %d, %d, %d" %(Duty_L1, Duty_L2, Duty_R1, Duty_R2)
	#if viewflag == 1:
	#	print "rad_ad_L = %03.3f rad_ad_R = %03.3f, t_rad = %03.3f, omL = %03.3f, omR = %03.3f" % (rad_ad_L, rad_ad_R, t_rad, om_ad_L, om_ad_R)

	Moter_L1_PWM.ChangeDutyCycle(Duty_L1)
	Moter_L2_PWM.ChangeDutyCycle(Duty_L2)
	Moter_R1_PWM.ChangeDutyCycle(Duty_R1)
	Moter_R2_PWM.ChangeDutyCycle(Duty_R2)




# メインループ
def main_loop():
	global viewflag
	PD = [0, 0]

	# エンコーダの値読み取り dt,cout_L, count_R
	raw = read_counter_from_arduino()
	# オドメトリ計算 車体[x],[y] 角度[rad]
	odo = calc_odometry(raw[0], (raw[1], raw[2]))
	# 移動判定 移動判定,目標距離、目標角度
	move = move_judgement((odo[0], odo[1]), odo[2], odo[3])

	# 回転判定なら回転さす
	if move[0] <= 2 :
		PD = calc_rotation(move[0], move[2], odo[2])
	elif move[0] == 3 or move[0] == 4 :
		PD = calc_straight(move[1], move[0])

	# PWM出力
	run_pwm(PD[0], PD[1], move[0], move[2], odo[4], odo[5])
	str_odo = "%03.3f %03.3f %03.3f\n" %(odo[0], odo[1], odo[2] * 180 / math.pi)
	#print "%03.3f %03.3f" %(PD[0], PD[1])
	if viewflag == 1:
		print str_odo
	f.write(str_odo)


	if move[0] == 6:
		move[0] == 0
		time.sleep(1)
		return 1
	else:
		return 0



# 次の目的地を探す
# 戻り値：tugipoint[x,y]
def nextpointsearch(client):
	global nowpos
	tugipoint = [0, 0]
	# 次のポイント設定
	# 戻り値：[一番いいポイント,二番目ポイント]
	nextpoint = move()

	# pipipipi号に最新データを送信
	#cltnew = ClientSetting('kuroki')
	SendData(client, 'node_data.txt', 'inputdata.txt')

	newpos = PosRead(client)	# 相手の現在地、目的地を取得
	pairnowpos = newpos[0:2]	# 相手の現在地
	pairtopos = newpos[2:]		# 相手の目的地

	print "相手の位置：",
	print pairnowpos, pairtopos
	print "nextpoint:",
	print nextpoint

	# 第一候補地に相手がいるか
	if nextpoint[0] == (pairnowpos or pairtopos):
		# 第二候補地にもいるならいったんその場待機
		if nextpoint[1] == (pairnowpos or pairtopos):
			tugipoint[0] = int(round(nowpos[0], 0))
			tugipoint[1] = int(round(nowpos[1], 0))
		# 空いてるなら第二候補地ぶち込み
		else:
			tugipoint = nextpoint[1]

	# 第一候補地にいないならそこへ移動
	else:
		tugipoint = nextpoint[0]

	return tugipoint




# メイン
def main():
	global nowpos, tp, firstp, viewtime, viewflag, oldtime
	try:
		# 初期設定
		print "起動中・・・"
		route = list()
		node = node_create()
		t_nowpos = [0,0]		# nowposの四捨五入した値が入る
		tonextpoint = [0,0]
		tp_old = [0, 0]
		oldtime = viewtime
		print "相方と接続中..."
		cltnew = ClientSetting('new')
		print "接続完了!"

		while True:
			# スタートとゴール設定
			# tuple型じゃないと受け付けてくれないので注意
			mapview()

			# 初期位置(nowpos=[0,0])にいる時スタート地点を手動設定
			if nowpos == [0,0]:
				print "スタート地点を設定してください 例）1,1"
				s = input()
				firstp = s
				nowpos[0] += firstp[0]
				nowpos[1] += firstp[1]
				t_nowpos = nowpos

			# 初期位置じゃなければ現在地(前回のゴール)をスタート地点に
			else:
				t_nowpos[0] = int(round(nowpos[0], 0))
				t_nowpos[1] = int(round(nowpos[1], 0))
				s = tuple(t_nowpos)

				#print "nowpos, (s)",
				#print nowpos,s

			# ゴール地点設定 初回は設定
			if tonextpoint == [0,0]:
				print "ゴール地点を設定してください   例) 3,1"
				g = input()
			# 次のよさげポイントが設定されてたらそこにする
			else:
				g = tuple(tonextpoint)

			#print 's,g:',
			#print s,g


			# スタートとゴールが一緒ならしばらく待機
			if s==g:
				print "最適ポジション。3秒待ちます"
				tp = s
				tp_old = s

				# 各種センサ読み込み・書き込み
				#print tp
				sensor_read(node, tp[0], tp[1])
				data_write(node)	# node_dataに書き込み
				data_read(node)		# 他から受け取ったデータを読み込み（更新もやってるよ）
				wait_pos = "%d %d"%(int(tp[0]), int(tp[1]))
				posdata = open('posdata.txt', 'w')
				posdata.write(wait_pos + " " + wait_pos)
				posdata.close()

				# 次の良さげポイントを検索
				tonextpoint = nextpointsearch(cltnew)

				# しばらくその場で待機
				time.sleep(3)
				continue

			# 経路取得、tpにぶち込む
			route = Astar(s,g)
			if len(route) != 0:
				tp = route[0]
				del route[0]
			else:
				print "スタートとゴールが同じ地点っぽい？"

			str_np = "%d %d"%(t_nowpos[0], t_nowpos[1])
			str_tp = "%d %d"%(tp[0], tp[1])
			posdata = open("posdata.txt", 'w')
			posdata.write(str_np + " " + str_tp)
			posdata.close()

			# すぐ始まるからちょっと間置くよ
			print "移動を開始します・・・"
			time.sleep(2)

			# 移動
			while True:
				#"""
				# mainloop内の各値を表示させる
				viewtime = time.time()
				if viewtime - oldtime > 0.1:
					viewflag = 1
					oldtime = viewtime
				#print viewflag, viewtime, oldtime
				#"""

				flag =main_loop()	# 目的地に着いたら1返す
				#print len(route)
				viewflag = 0

				# 目的地付近に着いたら次の目的地を指定
				if flag==1:
					# クソ電池食うのでPWM解放
					Moter_L1_PWM.ChangeDutyCycle(0)
					Moter_L2_PWM.ChangeDutyCycle(0)
					Moter_R1_PWM.ChangeDutyCycle(0)
					Moter_R2_PWM.ChangeDutyCycle(0)

					# 各種センサ読み込み・書き込み
					sensor_read(node, tp[0], tp[1])
					data_write(node)	# node_dataに書き込み
					data_read(node)		# 他から受け取ったデータを読み込み（更新もやってるよ）

					posdata = open('posdata.txt', 'w')
					posdata.write(str(tp[0]) + ' ' + str(tp[1]) + ' ' + str(tp[0]) + ' ' + str(tp[1]))
					posdata.close()
					#print "flag=1"
					#print route
					#data_show(node)		# データ確認用 表示に負荷かかるようならコメアウト

					# routeが空でなければ次の目的地をtpにぶち込む
					if len(route) != 0:
						tp_old = tp
						tp = list(route[0])
						tp[0] = int(tp[0])
						tp[1] = int(tp[1])
						del route[0]
						print "次の目的地：",
						print tp
						#time.sleep(1)

					# 空なら次のよさげポイントを設定
					else:
						tonextpoint = nextpointsearch(cltnew)

						#time.sleep(2)
						# 目的地設定フェーズへ
						flag = 0
						#print "flag=0"
						break


	except KeyboardInterrupt:
		arduino.close() # シリアルクローズ
		f.close()
		#posdata.close()
		GPIO.cleanup()
		closing(cltnew)
		print "End"
		sys.exit(1)

if __name__ == "__main__":
	main()

