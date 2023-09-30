from vpython import *
import json

file_name = "MCP_dt-0.65-lambda-0.5_Disturbance"
voltage = 8 #Which file to read
# with open(f"Bicycle_Reaction_Wheel\Data\DataToRead\{voltage}_Volts.json", mode='r') as file:
#     data_points = json.load(file)
# C:\Users\100655277\Desktop\Python\Research\Bicycle_Reaction_Wheel\Data\DataToRead\MPC_Data_1.json
with open(f"C:\\Users\\100655277\\Desktop\\Python\\Research\\Bicycle_Reaction_Wheel\\Data\\DataToRead\\{file_name}.json", mode='r') as file:
    data_points = json.load(file)

time = [data_point["Time"] for data_point in data_points]
velocity = [data_point["Velocity"] for data_point in data_points]
#Graphing
Graph1 = graph(title = "Velocity vs time", xtitle = "time [s]", ytitle = "Velocity [rpm]", width=800, height=400, fast = False)
#f__Desired_velocity = gcurve(color=color.red, label= "Desired Velocity")
f__Actual_velocity = gcurve(color=color.blue, label= "Actual Velocity")

for time, velocity in zip(time, velocity):  #The zip function is used to iterate through both lists simultaneously
    #rate(100) #number of calculations per second allowed
    f__Actual_velocity.plot(time, velocity)
    print(time,velocity)
    