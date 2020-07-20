#!/urs/bin/env python

import serial
from termcolor2 import c
import serial.tools.list_ports as lst
import time
import curses
import math

ser = serial.Serial()

logFile = open('/home/luca/Desktop/logFile_2.txt', 'w')

speed = 0
longitude = 0
latitude = 0
enc_speed = 0

def find_Stm():
    info = lst.comports()
    for port in info:
        if(port.product != None and port.product.find("STM32") != -1):
            return port.device
    return 0

def open_device(dev):
    ser.port = dev
    ser.baudrate = 2250000
    ser.open()
    ser.readline()

def parse_message(msg):
    msg = msg.decode('utf-8')
    msg = msg.replace("\r\n", "")
    msg = msg.split("\t")
    msg = msg[1:]
    return msg

def pick_message(msg):
    global speed, longitude, latitude, enc_speed
    if msg[0] == "208":
        if msg[1] == "16":

            buff = []
            for byte in msg:
                buff.append(int(byte))
            msg=buff

            longitude = msg[2] *256 + msg[3]
            longitude *= math.pow(2,16)
            longitude += msg[4] * 256 + msg[5]
            speed = msg[7]*256 + msg[8]
            speed /= 100

        if msg[1] == "17":
            buff = []
            for byte in msg:
                buff.append(int(byte))
            msg=buff
            latitude = msg[2] *256 + msg[3]
            latitude *= math.pow(2,16)
            latitude += msg[4] * 256 + msg[5]

        if msg[1] == '6':
            buff = []
            for byte in msg:
                buff.append(int(byte))
            msg=buff
            enc_speed = msg[2]*256 + msg[3]
        print("enc_speed: {} speed: {}, longitude: {}, latitude: {}\r\n".format(enc_speed, speed, longitude, latitude))
        logFile.write("enc_speed: {} speed: {}, longitude: {}, latitude: {}\r\n".format(enc_speed, speed, longitude, latitude))
        

if __name__ == "__main__":
    if find_Stm() != 0:
        open_device(find_Stm())
        print("Port Opened")
    else:
            print("no STM32 Detected, Exit_Program")
            exit(0)

    while True:
        try:
            msg = ser.readline()
            msg = parse_message(msg)
            #print(msg)
            pick_message(msg)
        except KeyboardInterrupt:
            logFile.close()
            exit(0)

ser.close()
