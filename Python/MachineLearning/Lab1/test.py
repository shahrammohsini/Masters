import pandas as pd
import numpy as np

# Assuming your data is stored in a file called "lp5.data"
with open("MachineLearning\Lab1\lp5.data", "r") as file:
    lines = file.readlines()

# Split the lines into rows and then split each row into 6 columns
data = []
current_row = []
matrix = np.zeros((2460,6))
rowCounter = 0
for line in lines:
    if line.strip():  # Check if the line is not empty
        # Split the line into 6 columns and convert them to integers
        rows = list(line.strip().split())
        if(len(rows) == 6):
            columnCounter = 0
            print("row: ", rowCounter)
            while columnCounter < 6:
                matrix[rowCounter, columnCounter]  = round(int(rows[columnCounter]),1)
                columnCounter = columnCounter + 1
            
            rowCounter = rowCounter + 1
            



    else:
        if current_row:
            data.append(current_row)
        current_row = []


print(matrix)
np.savetxt("output_matrix.csv", matrix, delimiter=",")
# Create a DataFrame with 6 columns
# df = pd.DataFrame(data, columns=["Column1", "Column2", "Column3", "Column4", "Column5", "Column6"])

# # Access the first column
# first_column = df["Column1"]

# print(first_column)
