import serial.tools.list_ports
from vpython import *
import time

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
Graph1 = graph(title = "Velocity vs time", xtitle = "time [s]", ytitle = "Velocity [rpm]", width=800, height=400, fast = False)
#f__Desired_velocity = gcurve(color=color.red, label= "Desired Velocity")
f__Actual_velocity = gcurve(color=color.blue, label= "Actual Velocity")


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
        
        #actual_vel = int(data.split(',')[0]) #Split data by "," and take the first value which is time
        #desired_vel = int(data.split(',')[1]) #Split data by "," and take the second value which is the desired velocity
        #actual_vel = int(data.split(',')[2]) #Take the 3rd value which is the actual velocity
        #elapsed_time = time.time() - start_time
        if len(str(data).split(',')) == 3 and str(data).split(',')[0]: #Ensuring there is enough data and no empty strings

            if python_timer_start is None: #this makes sure python_timer_start is updated only once as soon as the above if condition becomes true
                python_timer_start = time.time()

            # Calculate the elapsed time since the python_timer was recorded
            python_timer = time.time() - python_timer_start
            print(python_timer)

            timer = int(str(data.split(',')[0].strip()))
            desired_vel = int(str(data.split(',')[1].strip()))
            actual_vel = int(str(data.split(',')[2].strip()))
            f__Actual_velocity.plot(python_timer,actual_vel) #using python_timer instead because the time coming from arduino starts before python is ready to run the program
            #f__Desired_velocity.plot(timer,desired_vel)

        else:
            print("Not enough data")
            