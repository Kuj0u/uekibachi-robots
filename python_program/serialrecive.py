import serial
import time

s = serial.Serial('/dev/ttyACM0',115200, timeout = 10000)
data = 0
A = 0
B = 0
oldA = 0
oldB = 0
add = 0
count = 0

while 1 :
  data = s.readline()
#  if(data != 0) :
  print(data)

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
