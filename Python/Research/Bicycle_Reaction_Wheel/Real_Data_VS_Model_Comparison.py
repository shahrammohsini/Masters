import serial.tools.list_ports
from vpython import *
import time
import math
# Constants
t = 0 # Initial Time
dt = 0.2 # Time steps = 0.01 seconds
u = 10 #Voltage (Step input)
#k = 1.736*u + 206.3 # k for bike
#k = 217.6 average k for bike
#k = 15.4*u + 305.7 # For motor two   ----k = 403.7825 Average  for motor two
#k = 424.1263
#k = 7.4752*u + 371.8
#k = 15.942*u + 312.35 # Better data k = 426.521
k = 7.576*u + 400.05 # Better data from python k = 449.29
#k = 426.521
#tau = 0.0441*u + 2.4436 #tau for bike
#tau = 2.73 average tau for bike
#tau = -0.0311*u + 3.0273 #tau for motor two
#tau = 2.7417 #Average for motor two
#tau = 0.0003*u + 2.7394
#tau = -0.0255*u + 1.4509 # Better data tau = 1.273
#tau = 1.273
tau = -0.0455*u + 1.2455 # Better data from python tau = 0.927


e = math.e
Ymin = 0 # Y(n-1) this is velocity at previous step
Y = 0
python_timer_start = None

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
    val = 3 #Port number
    portVar = "COM" + str(val)
    serialInst = open_serial_port(portVar)
    if serialInst is not None:
        break





#Graphing
#Real Data
Graph1 = graph(title = "Real Velocity vs time", xtitle = "time [s]", ytitle = "Real Velocity [rpm]", width=800, height=400, fast = False)
f__Actual_velocity = gcurve(color=color.blue, label= "Actual Velocity")
#Model Data
#Graph2 = graph(title = "Simulated Velocity vs time", xtitle = "time [s]", ytitle = "Simulated velocity RPM", width=800, height=400, fast = False)
f_velocity = gcurve(color=color.red, label= "Sim Velocity")








#start_time = time.time()
run = "true"  #Send a message to Arduino to run the motor
serialInst.write(run.encode('utf-8'))


while True:
    if serialInst.in_waiting:  #read data as long as the serial port is open
        packet = serialInst.readline()


        #The data coming through is in unit code which is just intager representation of each character
        #print(packet.decode('utf')) #Decode back into string
        try:
            data = packet.decode('utf-8')
        except:
            data = 0

        if len(str(data).split(',')) == 3 and str(data).split(',')[0]: #Ensuring there is enough data and no empty strings
            
            if python_timer_start is None: #this makes sure python_timer_start is updated only once as soon as the above if condition becomes true
                python_timer_start = time.time()
            # Calculate the elapsed time since the python_timer was recorded
            python_timer = time.time() - python_timer_start

            #Real data cleaning
            print(data) #Split data by "," and take the first value which is time
            timer = int(str(data.split(',')[0].strip()))
            desired_vel = int(str(data.split(',')[1].strip()))
            actual_vel = int(str(data.split(',')[2].strip()))
            #Simulated function
            #Y = (k - (k/tau)*e**(-timer/tau))*u  # Y is angular velocity in time domain
            Y = (k*u*dt + tau*Ymin)/ (tau + dt)  # in discrete domain
            #Real data graph
            f__Actual_velocity.plot(python_timer,actual_vel)
            #Simulated Data Graph
            f_velocity.plot(t,Y)
            Ymin = Y
            t = t + dt
        else:
            print("Not enough data")
            
        
