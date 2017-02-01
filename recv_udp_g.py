# -*- coding: utf-8 -*-

import time
import socket
from sys import exit
from contextlib import closing

host = '127.0.0.1'  #受け取り側のIPアドレス
port = 4000   #適当なポート番号（このままでも大丈夫）
bufsize = 512    #バッファサイズ（お互い同じサイズで）

def uketori():
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  with closing(sock):
    sock.bind((host, port))
    list= sock.recv(bufsize).split("\t")
    recv_g = list[1]
    #print recv_g
  return recv_g


while 1 :
    a =uketori()
    print "受け取った値は : " + str(a)
