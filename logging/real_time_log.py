import serial
import serial.tools.list_ports

end=0

print("avaiable ports are:")
ports=list(serial.tools.list_ports.comports())
for p in ports:
	print(p)
	print("ciao")
serial_port=str(input("enter only the port number: "))
print("\nbaud?\n1) 9600\n2) 57600\n3) 115200\n4) 2250000\n5)other?")
serial_baud=input("enter: ")
if serial_baud=='1':
	serial_baud=9600
elif serial_baud=='2':
	serial_baud=57600
elif serial_baud=='3':
	serial_baud=115200
elif serial_baud=='4':
	serial_baud=2250000
else:
	serial_baud=input("digit the baud: ")
file_name=input("file_name?(without .txt): ")
file_log=open(file_name+".txt",'w');
print("opening COM"+serial_port+" at baud="+str(serial_baud)+" ....",end='\n')
ser = serial.Serial("COM"+serial_port,int(serial_baud))
ser.close()
ser.open()
print("port is open correctly")
while end == 0:
	line=ser.readline()
	line_str=line.decode('ascii')
	line_str=line_str[0:len(line_str)-2]
	file_log.write(line_str+"\n")
	print(line_str)
	file_log.close()
	file_log=open(file_name+".txt",'a');
file_log.close()
print("end\n")
ser.close()



