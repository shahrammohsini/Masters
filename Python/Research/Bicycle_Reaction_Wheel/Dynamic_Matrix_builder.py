import json
import copy

voltage = 4 #Which file to read
with open(f"Bicycle_Reaction_Wheel\Data\Motor_Two_Data_For_Comparison\{voltage}_Volts.json", mode='r') as file:
    data_points = json.load(file)
velocity_Array = [data_point["Velocity"] for data_point in data_points]
# velocity_Array = [1,2,3,4,5,6,7,8,9]
N = 3
r = len(velocity_Array) #get number of rows
#A = [1,2,3,4,5,6,7,8,9]
Mod_Array = copy.deepcopy(velocity_Array)
for i in range(N-1):
    Mod_Array.insert(0,0)  # This inserts N-1 zeros into array Mod_Array


C = N #get number of columns
m = [] #create matrix (array of arrays)

for i in range (r):
    E = []
    for j in range(C):
        v = Mod_Array[(i - 1 + (N - j))]  #formula to fill dynamic matrix appropriately.
        E.append(v)
    m.append(E)


print(m)

#save the data in a json file
with open(f"Bicycle_Reaction_Wheel\Data\Dynamic_Matrix\Matrix.json", mode="w") as file:
    json.dump(m, file, indent=5) #The indent makes it esier to read if you just open the json file
file.close()
