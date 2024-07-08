import math
import json
import copy
# Constants
Ymin = 0
t = 0 # Initial Time
dt = 0.01 # Time steps = 0.01 seconds
u = 12.2 #Voltage (Step input)

e = math.e
Y = 0

velocity_Array = []




#Simulation
while t < 1: # Run simulation for 1 sec
    Y = (k*u*dt + tau*Ymin)/ (tau + dt)  # in discrete domain
    velocity_Array.append(Y/u) # Fill this array with data for the dynamic matrix. To normalize divide by u
    t = t + dt
    Ymin = Y
    

#create dynamic matrix

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
with open(f"Motor_Two\Data\Dynamic_Matrix\Matrix_Sim.json", mode="w") as file:
    json.dump(m, file, indent=5) #The indent makes it esier to read if you just open the json file
file.close()




