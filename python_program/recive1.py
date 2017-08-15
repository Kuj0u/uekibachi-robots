import serial
import time

s = serial.Serial('/dev/ttyACM0',230400, timeout = 1000)
dataL = 0
dataR = 0
A = 0
B = 0
oldA = 0
oldB = 0
addL = 0
addR = 0
countL = 0
countR = 0

while 1 :
  data = int(s.readline())
  addL = (3 & data) - 2
  addR = (3 & (data / 100)) - 2
  countL = countL + addL
  countR = countR + addR
#  if(data != 0) :
  print("countL = %5d, countR = %5d" % (countL, countR))

#  if(oldA == oldB) :
#    if(oldA != A) :
#      add = 1
#    else :
#      add = -1
#  else :
#    if(oldA == A) :
#      add = 1
#    else :
#      add = -1
#
#  count = count + add

#  print('A='+A+' B='+B+' oldA='+str(oldA)+' oldB='+str(oldB)+' add='+str(add)+' count='+str(count))

#  oldA = A
#  oldB = B
