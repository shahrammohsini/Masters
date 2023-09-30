# from vpython import *
# import json
# import os

# Graph1 = graph(title = "Velocity vs time", xtitle = "time [s]", ytitle = "Velocity [rpm]", width=800, height=400, fast = False)

# directory_path = "C:\\Users\\100655277\\Desktop\\Python\\Research\\Bicycle_Reaction_Wheel\\Data\\DataToRead\\"
# file_names = [f for f in os.listdir(directory_path)]
# print(file_names)
# for file_name in file_names:
#     with open(f"C:\\Users\\100655277\\Desktop\\Python\\Research\\Bicycle_Reaction_Wheel\\Data\\DataToRead\\{file_name}", mode='r') as file:
#         data_points = json.load(file)
    
#     time = [data_point["Time"] for data_point in data_points]
#     velocity = [data_point["Velocity"] for data_point in data_points]

#     f__Actual_velocity = gcurve(color=color.blue, label= "Actual Velocity")

#     for time, velocity in zip(time, velocity):  #The zip function is used to iterate through both lists simultaneously
#         #rate(100) #number of calculations per second allowed
#         f__Actual_velocity.plot(time, velocity)
#         print(time,velocity)



# from vpython import *
# import json
# import os

# Graph1 = graph(title="Velocity vs time", xtitle="time [s]", ytitle="Velocity [rpm]", width=800, height=400, fast=False)

# # List of vpython colors
# colors = [
#     color.red, color.green, color.blue, color.yellow, color.orange, color.cyan, color.magenta, 
#     color.white, color.black, color.gray, vec(0.5, 0.2, 0.1), vec(0.2, 0.5, 0.1), 
#     vec(0.2, 0.1, 0.5), vec(0.8, 0.4, 0.7)
# ]


# directory_path = "C:\\Users\\100655277\\Desktop\\Python\\Research\\Bicycle_Reaction_Wheel\\Data\\DataToRead\\"
# file_names = [f for f in os.listdir(directory_path)]
# print(file_names)

# # Make sure we don't run out of colors
# assert len(file_names) <= len(colors), "Too many files for the available colors."

# for index, file_name in enumerate(file_names):
#     with open(os.path.join(directory_path, file_name), mode='r') as file:
#         data_points = json.load(file)

#     time = [data_point["Time"] for data_point in data_points]
#     velocity = [data_point["Velocity"] for data_point in data_points]

#     # Use the filename (without extension) as the label and select color based on index
#     f__Actual_velocity = gcurve(color=colors[index], label=os.path.splitext(file_name)[0])

#     for t, v in zip(time, velocity):
#         #rate(100) #number of calculations per second allowed
#         f__Actual_velocity.plot(t, v)
#         print(t, v)


from vpython import *
import json
import os

# Combined graph
Graph1 = graph(title="Velocity vs time (Combined)", xtitle="time [s]", ytitle="Velocity [rpm]", width=1400, height=800, fast=False)

# List of vpython colors
colors = [
    color.red, color.green, color.blue, color.yellow, color.orange, color.cyan, color.magenta, 
    color.black, color.gray, vec(0.5, 0.2, 0.1), vec(0.2, 0.5, 0.1), 
    vec(0.2, 0.1, 0.5), vec(0.8, 0.4, 0.7)
]

directory_path = "C:\\Users\\100655277\\Desktop\\Python\\Research\\Bicycle_Reaction_Wheel\\Data\\DataToRead\\ThisFolder"
file_names = [f for f in os.listdir(directory_path)]
print(file_names)

# Make sure we don't run out of colors
assert len(file_names) <= len(colors), "Too many files for the available colors."

for index, file_name in enumerate(file_names):
    with open(os.path.join(directory_path, file_name), mode='r') as file:
        data_points = json.load(file)

    time = [data_point["Time"] for data_point in data_points]
    velocity = [data_point["Velocity"] for data_point in data_points]

    # Separate graph for each file
    Graph_individual = graph(title=f"Velocity vs time ({os.path.splitext(file_name)[0]})", xtitle="time [s]", ytitle="Velocity [rpm]", width=1200, height=800, fast=False)
    
    # Curve for the individual graph
    f_individual = gcurve(graph=Graph_individual, color=colors[index], label=os.path.splitext(file_name)[0])
    
    # Curve for the combined graph
    f_combined = gcurve(graph=Graph1, color=colors[index], label=os.path.splitext(file_name)[0])

    for t, v in zip(time, velocity):
        #rate(100) #number of calculations per second allowed
        f_individual.plot(t, v)
        f_combined.plot(t, v)
        print(t, v)
