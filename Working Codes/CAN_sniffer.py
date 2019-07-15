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
    msg = msg.replace("\r\n", "")
    msg = msg.split("\t")
    return msg

def message_append(messages, msg, time):

    msg[0] = time

    if len(messages) > 0:
        for i in range(len(messages)):
            if messages[i][2] == msg[1]:
                dt = int((time - float(messages[i][1]))*1000000)/1000
                msg = [dt] + msg
                messages[i] = msg
                return messages
            else:
                continue

    msg = [time] + msg
    messages.append(msg)

    return messages

def print_messages(list_):
    print("\r\n"*100)
    for i in range(len(list_)):
        # string = ""
        # for num in list_[i]:
        #     string += str(num) + " \t"
        # for j in range(i):
        #     print("\x1b[A", end = (""))
        # print(c(string, "white"))
        for num in list_[i]:
            print(str(num) + "\t", end="")
        print("")
        #print(list_[i])

if __name__ == "__main__":
    if find_Stm() != 0:
        open_device(find_Stm())
        print("Port Opened")
    else:
            print("no STM32 Detected, Exit_Program")
            exit(0)

    message_list = []
    analisys_duration = 1

    #print(c("Start SNIFFING COCAINE = LA DROGA").blink.red.underline)
    start_time = time.time()
    secondary_timer = 0
    while True:
        current_sec = round(time.time() - start_time, 5)

        msg = str(ser.readline(), 'ascii')
        msg = parse_message(msg)
        message_list = message_append(message_list, msg, current_sec)

        if current_sec - secondary_timer > 1:
            print_messages(message_list)
            secondary_timer = current_sec
        #print(message_list[0][2])

ser.close()
