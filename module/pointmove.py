#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import time
import os
#from module import data_write, data_show

# Astarアルゴリズムで次に移動するポイントを決める
# 戻り値：goal
def move():
	# [x,y,number]
	best_light = [0, 0, 0]
	best_water = [0, 0, 0]
	best_humidity = [0, 0, 0]
	best_temperatur = [0, 0, 0]
	best_evaluation = [0, 0, 0]
	evaluation = 0
	next_position = [0, 0]
	second_position = [0,0,0]

	# node_data.txtが無かったら作っておく　無いと怒られる
	#if not os.path.exists('node_data.txt'):
	#	print "node_dataがありません。作成します"
	#	data_write()

	# node情報読み込み
	node = open('node_data.txt', 'r')
	line = node.readlines()
	node.close()

	# 各行ごとに分解 [x,y,state,light,water,humidity,temperatur,time]
	for mario in line:
		mario = mario.split()
		#print mario
		x = int(mario[0])
		y = int(mario[1])

		# 通行可能で数値が入ってるなら、最大値か確認
		if "false" not in mario and "None" not in mario:
			if best_light[2] < mario[3]:
				li_temp = float(mario[3]) * 1000
				best_light = [x, y, li_temp]
			if best_water[2] < mario[4]:	# 水分量は評価値関係ないかも
				wa_temp = float(mario[4]) * 1000
				best_water = [x, y, wa_temp]
			if best_humidity[2] < mario[5]:
				hu_temp = float(mario[5]) * 100
				best_humidity = [x, y, hu_temp]
			if best_temperatur[2] < mario[6]:
				te_temp = float(mario[6])
				best_temperatur = [x, y, te_temp]

			# 評価値書き込み　式は適当に全乗算
			evaluation = li_temp + hu_temp + te_temp
			# 一番よさげポイントならbestpoint更新 旧bestpointは二番目に
			if evaluation > best_evaluation[2]:
				second_position = best_evaluation[:3]
				best_evaluation = [x,y,evaluation]
			# 一番じゃないけど二番目よりよさげなら二番目更新
			elif evaluation > second_position[2]:
				second_position = [x,y,evaluation]
			print "second_position:",
			print second_position
			print "best_evaluation:",
			print best_evaluation



	next_position = [best_evaluation[0], best_evaluation[1]]

	# 各数値表示（デバッグ）
	print "light      : {0}".format(best_light)
	print "water      : {0}".format(best_water)
	print "humidity   : {0}".format(best_humidity)
	print "temperatur : {0}".format(best_temperatur)
	print "evaluation : {0}".format(best_evaluation)
	print "next       : {0}".format(next_position)
	print "second     : {0}".format(second_position)

	return (next_position, second_position[:2])

"""
try:
	#data_show()
	granblue = move()

except KeyboardInterrupt:
	print "\nend\n"
	sys.exit(1)
"""
