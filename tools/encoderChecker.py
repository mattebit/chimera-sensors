#!/usr/bin/env python3

# Python tool to display encoder data graphically
# Needs to receive from Uart 3 numbers separated by ","
# angle0,angle1,speed

import sys
import serial
import serial.tools.list_ports as lst
import time
import curses
import cv2
import math
import numpy as np
import random

W, H = 800, 512

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

def parse_message(msg):
    msg = msg.replace("\r\n", "")
    msg = msg.split(",")
    buff = []
    for i, el in enumerate(msg):
        buff.append(float(el)/100)
    return buff

def display_enc_data(a0, a1, speed, da):
    image = np.zeros((H,W,3), np.uint8)
    image[:,:] = (50,50,50)

    radius = 100
    circleCenter = (int(W/3), int(H/2))
    image = cv2.circle(image, circleCenter, radius, (250,250,250),1)

    point = (int(circleCenter[0] + radius*math.cos(math.radians(a0))),
            int(circleCenter[1] + radius*math.sin(math.radians(a0))))
    image = cv2.line(image, circleCenter, point, (125,125,0), 1)

    point = (int(circleCenter[0] + radius*math.cos(math.radians(a1))),
            int(circleCenter[1] + radius*math.sin(math.radians(a1))))
    image = cv2.line(image, circleCenter, point, (125,125,0), 1)

    radius = 100
    circleCenter = (int(W*2/3), int(H/2))
    image = cv2.circle(image, circleCenter, radius, (250,250,250),1)

    point = (int(circleCenter[0] + radius*math.cos(0)),
            int(circleCenter[1] + radius*math.sin(0)))
    image = cv2.line(image, circleCenter, point, (125,125,0), 1)

    point = (int(circleCenter[0] + radius*math.cos(math.radians(da))),
            int(circleCenter[1] + radius*math.sin(math.radians(da))))
    image = cv2.line(image, circleCenter, point, (125,125,0), 1)

    cv2.imshow("Encoder Data", image)
    cv2.waitKey(1)

device = find_Stm()
if not device == 0:
    open_device(device)
    print("found STM")
else:
    print("No STM found")
    exit(0)

while True:
    msg = str(ser.readline(), 'ascii')
    #try:
    msg = parse_message(msg)
    if len(msg) >= 3:
        display_enc_data(msg[0], msg[1], msg[2], msg[3])
        #print("speed {}, delta angle {}".format(msg[2], msg[3]))
    # except:
    #     print("error ", sys.exc_info()[0])