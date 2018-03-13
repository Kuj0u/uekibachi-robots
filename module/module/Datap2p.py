# -*- coding:utf-8 -*-

# データの送受信を行うプログラムです　これ単体では動きません
# 研究用ロボットに最適化してあるので流用する際は改変ﾋｯｽです！
# DataUpdate()をコメントアウトすると動くかもしれません

# モジュール import ~
import sys
import paramiko
import scp
import os

remortPath = None

class Local:
	def __init__(self):
		self.path = os.getcwd()	# デフォルトのPath

# 通信設定
# 引数：送り先[str]
# 戻り値：[sftpクライアント, scpクライアント]
def ClientSetting(to):
	global clientdata, remortPath
	ssh = None
	sc = None
	sftp = None
	local = Local()

	# paramiko
	ssh = paramiko.SSHClient()
	ssh.load_system_host_keys()
	ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

	if to in ['new', 'n']:
		ssh.connect('192.168.0.90', port=22, username='pipipipi', password='oppai')
		remortPath = '/home/pipipipi/'
	elif to in ['kuroki', 'k']:
		ssh.connect('192.168.0.81', port=22, username='pipipi', password='oppai')
		remortPath = '/home/pipipi/'
	elif to in ['reba', 'r']:
		ssh.connect('192.168.0.12', port=22, username='REBA_Ubuntu', password='rikamoe01')
		remortPath = '/home/REBA_Ubuntu/'

	sftp = ssh.open_sftp()

	# scp
	sc = scp.SCPClient(ssh.get_transport())

	print ('ローカルパス：{}'.format(local.path))
	print ('接続先：{}'.format(remortPath))
	return (sftp, sc, ssh)


# 最も新しいデータをデータに追加する
def DataUpdate():
	# 製作中
	pass

# データの送信
# 引数：送りたいデータ[str], 送った後の名前[str]
def SendData(clientdata, senddata, changedname):
	global remortPath
	client = clientdata[0]
	remort = remortPath
	DataUpdate()

	remort += changedname # 送った後の名前設定

	print ('送るファイル：{}'.format(senddata))
	print ('送るpathと名前：{}'.format(remort))

	client.put(senddata, remort)

# データの受信
# 引数：受け取りたいデータ[str], 受け取った後の名前[str]
def ReciveData(clientdata, remortpath, recivename):
	client = clientdata[1]
	local = Local()
	remort = remortPath
	DataUpdate()

	remort += remortpath
	print ('受け取るファイル：{}'.format(remort))
	print ('受け取ったファイルの名前：{}'.format(recivename))

	client.get(remort, recivename)

# 相手のnowpos（現在地）を読み取る
# 戻り値：相手の場所[x,y]
def PosRead(clientdata):
	position = list()
	client = clientdata[2]
	stdin,stdout,stderr = client.exec_command('cat posdata.txt')
	temp = stdout.read()
	position = temp.split()
	position[0] = int(position[0])
	position[1] = int(position[1])

	return position

# 後処理
def closing(cli):
	global remortPath
	remortPath = None
	cli[0].close()
	cli[1].close()
	#sys.exit(1)

# メインループ
#def main_loop():

# メイン
"""
def main():
	client = ClientSetting()
	try:
		#while True:
		#	main_loop()

		# Debug
		SendData()
		ReciveData()

	except KeyboardInterrupt:
		closing()
		sys.exit(1)
		print "\nend\n"
	finally:
		closing()

if __name__ == "__main__":
	main()
"""
