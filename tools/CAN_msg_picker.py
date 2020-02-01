#!/urs/bin/env python

import serial
from termcolor2 import c
import serial.tools.list_ports as lst
import time
import curses

ser = serial.Serial()

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
    if msg[0] == "208":
        if msg[1] == "18":
            for byte in msg[2:]:
                print(str(chr(int(byte))),end=" ")
            print("")

if __name__ == "__main__":
    if find_Stm() != 0:
        open_device(find_Stm())
        print("Port Opened")
    else:
            print("no STM32 Detected, Exit_Program")
            exit(0)

    while True:
        msg = ser.readline()
        msg = parse_message(msg)

        pick_message(msg)

ser.close()
