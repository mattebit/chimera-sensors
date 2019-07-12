#!/urs/bin/env python

import serial
import serial.tools.list_ports as lst
import time

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


def message_append(messages, msg):
    if len(messages) > 0:
        for i in range(len(messages)):
            couple = messages[i]
            if couple[0] == msg[1]:
                couple = (couple[0], couple[1] + 1)
                messages[i] = couple
                return messages
            else:
                continue

    messages.append((msg[1], 1))

    return messages

if __name__ == "__main__":
    print("main")
    if find_Stm() != 0:
        open_device(find_Stm())
    else:
            print("no STM32 Detected, Exit_Program")
            exit(0)

    message_list = []
    analisys_duration = 1

    print("Start analizing CAN messages")
    while True:
        msg = str(ser.readline(), 'ascii')
        msg = parse_message(msg)
        message_list = message_append(message_list, msg)
        print(message_list)

ser.close()
