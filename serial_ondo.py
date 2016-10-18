import serial
import time

def main():
    con=serial.Serial('/dev/ttyAMA0', 9600, timeout=10)
    print con.portstr
    while 1:
        str=con.readline()
        print str
        con.write(str)

if __name__ == '__main__':
    main()
