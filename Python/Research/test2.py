import serial.tools.list_ports
from vpython import *
import time


# Function to open the serial port with error handling
def open_serial_port(port):
    try:
        serialInst = serial.Serial(port, baudrate=9600)
        return serialInst
    except serial.SerialException:
        print(f"Failed to open port {port}. Retrying...")
        return None

ports = serial.tools.list_ports.comports()
portList = [str(onePort) for onePort in ports]
print("\n".join(portList))

while True:
    #val = input("Select port: COM ")
    val = 3
    portVar = "COM" + str(val)
    serialInst = open_serial_port(portVar)
    if serialInst is not None:
        break




#Graphing
Graph1 = graph(title = "Velocity vs time", xtitle = "time [s]", ytitle = "Velocity [m/s]", width=400, height=200, fast = True)
f_velocity = gcurve(color=color.purple, label= "Velocity")

start_time = time.time()

while True:
    if serialInst.in_waiting:  # read data as long as the serial port is open
        packet = serialInst.readline()
        try:
            data = packet.decode('utf-8')
        except UnicodeDecodeError:
            try:
                data = packet.decode('ascii')
            except UnicodeDecodeError:
                try:
                    data = packet.decode('latin-1')
                except UnicodeDecodeError:
                    data = packet.decode('utf-16')
        
        #desired_vel = int(data.split(',')[0])  # Split data by "," and take the first value which is the desired velocity
        #actual_vel = int(data.split(',')[1])  # Take the second value which is the actual velocity
        print(int(str(data.split(',')[0]).strip())) #
        elapsed_time = time.time() - start_time
        f_velocity.plot(elapsed_time, 1)