import time
import serial

ser = serial.Serial()

ser.port = "/dev/ttyACM1"
ser.baudrate = 115200

ser.open()

log = open('logs/log', 'w')
while True:
  line = str(time.time()) + ";" + ser.readline().decode() 
  log.write(line)
  print(line, end='')
