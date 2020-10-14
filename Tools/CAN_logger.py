import serial
import serial.tools.list_ports as lst
import time

logFile = open('/home/filippo/Desktop/logFile_1.txt', 'w')

info = lst.comports()

ser = serial.Serial()

def find_Stm():
    for port in info:
        if(port.product.find("STM32") != -1):
            return port.device
    return 0

def open_device(dev):
    ser.port = dev
    ser.baudrate = 2250000
    ser.open()
    ser.readline()


def parse_message(msg):
    msg = msg.decode('utf-8')
    msg = msg.split("\t")[1:]
    msg = "\t".join(msg)
    return msg


if __name__ == "__main__":
    if find_Stm() == 0:
        print("no STM32 Detected, Exit_Program")
        exit(0)

    open_device(find_Stm())


    while True:
        try:
            msg = ser.readline()
            msg = parse_message(msg)
            msg = str(time.time()) + "\t" + msg
            logFile.write(msg)
        except KeyboardInterrupt:
            logFile.close()
            exit(0)

    
