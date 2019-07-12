#!/urs/bin/env python

import serial
import serial.tools.list_ports as lst
import time

#info = lst.comports()

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

def message_counter(messages, msg):
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

def analize_data(data):
    total_msg = 0
    for couple in data:
        total_msg += couple[1]

    percentages = []

    for couple in data:
        percent = (couple[1] / total_msg) * 100
        percentages.append((couple[0], percent))

    return total_msg, percentages


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
    start_time = time.time()
    while time.time() - start_time < analisys_duration:
        msg = str(ser.readline(), 'ascii')
        msg = parse_message(msg)
        message_list = message_counter(message_list, msg)

    print("Enlapsed " + str(analisys_duration) + " seconds\n\n")

    total, list_percent = analize_data(message_list)

    total_lines = 50
    span = 5
    print("=" * total_lines)
    print("-" * int((total_lines-14)/2) + "RETRIEVED DATA" + "-" * int((total_lines-14)/2))
    print("=" * total_lines + "\n")


    # TOTAL MESSAGES
    print("-"*total_lines)
    txt = "TOTAL Messages"
    print(txt + " " * int(total_lines- len(txt) - span) + str(total))
    print("-"*total_lines)

    # BYTES TRANSMITTED
    bytes_transmitted = total * 8 * 8 + total * 14
    bytes_transmitted = round(bytes_transmitted / 1000000, 3)
    txt = "Transmitted"
    print(txt + " " * int(total_lines- len(txt) - span) + str(bytes_transmitted) + " Mb")
    print("_"*total_lines)

    # CAN SPEED
    CAN_Speed = bytes_transmitted / analisys_duration
    CAN_Speed = round(CAN_Speed, 3)
    txt = "CAN Speed"
    print(txt + " " * int(total_lines- len(txt) - span) + str(CAN_Speed) + " Mb/s")
    print("-"*total_lines)

    # AVERAGE TIME DELTA
    average_delta = analisys_duration / total
    average_delta = round(average_delta * 1000, 4)
    txt = "Average time delta"
    print(txt + " " * int(total_lines- len(txt) - span) + str(average_delta) + " ms")
    print("-"*total_lines)

    # MESSAGES PER SECOND
    msg_per_sec = total / analisys_duration
    msg_per_sec = round(msg_per_sec, 2)
    txt = "Messages per second"
    print(txt + " " * int(total_lines- len(txt) - span) + str(msg_per_sec) + " Hz")
    print("-"*total_lines)

    print("\n")
    print("id Percentual in total messages")

    print("\tID\tPERCENT")
    for idx in list_percent:
        print ("\t" + str(idx[0]) + "\t" + str(round(idx[1], 4)))

ser.close()
