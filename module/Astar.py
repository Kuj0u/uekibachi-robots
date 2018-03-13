#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys

"""
map_data = [
'00000000',
'0S0    0',
'0 0 0  0',
'0   0  0',
'0   0 G0',
'00000000',
]
"""
# map_dataの読み込み
map_data = list()
f = open('map_data.txt', 'r')

for line in f:
	map_data.append(line[:-1])
f.close()
#print map_data

map_width = max([len(x) for x in map_data])
map_height = len(map_data)

class Node(object):
	"""
	f(n) ... startからgoalまでの最短距離
	g(n) ... startからnノードまでの最短距離
	h(n) ... nノードからgoalまでの最短距離
	f(n) = g(n) + h(n)

	関数を推定値にすることにより最短経路を予測する
	h*(n)をn～goalまでの直線距離と仮定する。
	"""
	start = None
	goal = None

	def __init__(self, x, y):
		self.pos		= (x,y)
		self.hs			= (x - self.goal[0])**2 + (y-self.goal[1])**2
		self.fs			= 0
		self.owner_list	= None
		self.parent_node= None

	def isGoal(self):
		return self.goal == self.pos

class NodeList(list):
	def find(self, x, y):
		l = [t for t in self if t.pos==(x,y)]
		return l[0] if l != [] else None

def Astar(start, goal):

	route = list()
	# スタート地点とゴール地点設定 引数から持ってくる
	# 現時点ではmap_dataからstart,goal持ってくるようになってる
	Node.start = start
	Node.goal = goal

	#OpenリストとCloseリスト設定
	open_list		= NodeList()
	close_list		= NodeList()
	start_node		= Node(*Node.start)
	start_node.fs	= start_node.hs
	open_list.append(start_node)

	while True:
		# Openリストが空になったら解無し
		if open_list == []:
			print "エラーっぽい？"
			sys.exit(1)

		# Openリストからf*が最小のノードnを取得
		n = min(open_list, key=lambda x:x.fs)
		open_list.remove(n)
		close_list.append(n)

		# 最小ノードだたら終了
		if n.isGoal():
			end_node = n
			break

		# f*() = g*() + h*() -> g*() = f*() - h*()
		n_gs = n.fs - n.hs

		# ノードnの移動可能方向のノードを調べる
		for v in ((1,0), (-1,0), (0,1), (0,-1)):
			x = n.pos[0] + v[0]
			y = n.pos[1] + v[1]

			# mapが範囲外または壁(0)の場合はcontinue
			if not(0 < y < map_height and 0 < x < map_width and map_data[y][x] != '0'):
				continue

			# 移動先のノードがOpen,Closeのどちらかのリストに
			# 格納されているか、または新規ノードなのかを調べる
			m = open_list.find(x, y)
			dist = (n.pos[0]-x)**2 + (n.pos[1]-y)**2
			if m:
				# 移動先のノードがOpenリストに格納されていた場合、
				# より小さいf*ならばノードmのf*を更新し、親を書き換え
				if m.fs > n_gs + m.hs + dist:
					m.fs > n_gs + m.hs + dist
					m.parent_node = n
			else:
				m = close_list.find(x,y)
				if m:
					# 移動先のノードがcloseリストに格納されていた場合、
					#より小さいf*ならばノードmのf*を更新し、親を書き換え
					# かつ、Openリストに移動する
					if m.fs > n_gs + m.hs + dist:
						m.fs = n_gs + m.hs + dist
						m.parent_node = n
						open_list.append(m)
						close_list.remove(m)
				else:
					# 新規ノードならばOpenリストにノード追加
					m = Node(x,y)
					m.fs = n_gs + m.hs + dist
					m.parent_node = n
					open_list.append(m)

	# endノードから親をたどっていくと、最短ルートがわかる
	n = end_node.parent_node
	m = [[x for x in line] for line in map_data]
	route.append(goal)

	while True:
		route.append(n.pos)
		if n.parent_node == None:
			break
		m[n.pos[1]][n.pos[0]] = '+'
		n = n.parent_node

	m[start[1]][start[0]] = 'S'
	m[goal[1]][goal[0]] = 'G'

	# 経路表示
	for yu in range(map_width):
		if yu >= 10:
			yu = yu % 10
		sys.stdout.write(str(yu))
	print " "
	print "\n".join(["".join(x) for x in m])

	# routeに経路返すよ
	route.reverse()
	print route

	return route

def mapview():
	for boss in range(map_width):
		if boss >= 10:
			boss = boss % 10
		sys.stdout.write(str(boss))
	print " "
	print "\n".join(map_data)
"""
def main():
	try:
		while True:
			mapview()
			s = input()
			g = input()
			Astar(s,g)
	except KeyboardInterrupt:
		print "\nend"
		sys.exit(1)

if __name__ == "__main__":
	main()
"""
