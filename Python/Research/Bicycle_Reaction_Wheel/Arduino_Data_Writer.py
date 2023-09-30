import serial.tools.list_ports
from vpython import *
import time
import json


#Create an empty list to save data
data_points = []


voltage = 4 #Voltage applied to motor
duration = 25 #how long to run the sim in seconds
python_timer_start = None #this timer will be used as the time for the time vs velocity. This is more correct than the arduino bc arduion timer starts before python is ready to collect data

#get the current time at the start of the loop.
start_time = time.time() #This timer is just used to make sure the program runs for as long as I want it to.

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

            #print(data) #Split data by "," and take the first value which is time
            timer = int(str(data.split(',')[0].strip()))
            desired_vel = int(str(data.split(',')[1].strip()))
            actual_vel = int(str(data.split(',')[2].strip()))
            f__Actual_velocity.plot(python_timer,actual_vel)
            #f__Desired_velocity.plot(timer,desired_vel)
            print(python_timer, actual_vel)


            try:
                #append the data into the list
                data_points.append({"Time":python_timer, "Velocity":actual_vel})
                # Create a file to save this data
                with open(f"Bicycle_Reaction_Wheel\Data\Motor_Two_Data_For_Comparison\{voltage}_Volts.json", mode="w") as file:
                    json.dump(data_points, file, indent=1) #The indent makes it esier to read if you just open the json file
                file.close()
            except Exception as e:
                print(f"Error while writing data to the file: {e}")
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                break #exit the simulation after 25 sec
                
        else:
            print("Not enough data")
            

print("Simulation complete")