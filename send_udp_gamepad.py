import pygame
import time
import math
from pygame.locals import *
from sys import exit
#from __future__ import print_function
import socket
from contextlib import closing

pygame.init()
clock = pygame.time.Clock()
joystick = None



if pygame.joystick.get_count()>0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

if joystick is None:
    print "No joysticks"
    exit()


def okuru(joy_Lx, joy_Ly, joy_Rx, joy_Ry):
  host = '192.168.0.81'
  port = 4000
  #count = 0
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  with closing(sock):
    #while True:
      #message = 'Hello world : {0}'.format(count).encode('utf-8')
      message ='%f,%f,%f,%f' %(joy_Lx, joy_Ly, joy_Rx, joy_Ry) 
      print(message)
      sock.sendto(message, (host, port))
      #count += 1
      print message
      time.sleep(0.1)
  return


while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            exit()
    if joystick.get_numaxes()>=2:
        axis_Lx = joystick.get_axis(0)
        axis_Ly = joystick.get_axis(1)
        axis_Rx = joystick.get_axis(2)
        axis_Ry = joystick.get_axis(3)
    okuru(axis_Lx, axis_Ly, axis_Rx, axis_Ry)
