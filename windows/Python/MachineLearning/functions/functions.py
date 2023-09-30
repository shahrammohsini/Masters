import pandas as pd
from sklearn.model_selection import train_test_split
import os
import numpy as np



class Setup():

    '''
    function Name: getData
    Arguments: None
    Purpose: get data and name columns from one to number of columns. ex: column_1, column_2 ...column_N
    '''
    def getData(self):
        # file_names = os.listdir(directory_path)
        # file_names = os.listdir(directory_path)
        # # Print the names of files in the directory
        # for file_name in file_names:
        #     print(file_name)

        data_file = input("Enter file name: ")
        num_columns = len(pd.read_csv(f'/Users/100655277/Desktop/Python/MachineLearning/Lab1/{data_file}.data').columns)

        #create column names
        column_names = [f'column_{i}' for i in range(1, num_columns+1)]
        #read csv file into a data frame, and name columns
        df = pd.read_csv(f'/Users/100655277/Desktop/Python/MachineLearning/Lab1/{data_file}.data', names = column_names)

        return df

    '''
    function Name: getMoveColumnToEndData
    Arguments: 
                data_frame: Data to be edited
    Purpose: Ask for a column name and move it to the end
    '''
    def moveColumnToEnd(self, data_frame):
        column_name = input("Which column do you want to move?  ")

        #Remove the desired column from df and save it to column_data
        column_data = data_frame.pop(column_name)
        #Add Diagnosis back into df
        data_frame[column_name] = column_data   

        #print(data_frame) 
        return data_frame

    
    
    '''
    function Name: MoveColumn
    Arguments: 
                data_frame: Data to be edited
    Purpose: Ask for a column name and move it
    '''
    def moveColumn(self, data_frame):
        column_name = input("Which column do you want to move? ex: 'column_1' : ")
        num_columns = len(data_frame.columns)
        New_Col_position = input(f"Where do you want to move your column from 1 to {num_columns}?  ")

        #Remove the desired column from df and save it to column_data
        print("removing column")
        column_data = data_frame.pop(column_name)
        #Add Diagnosis back into df
        data_frame.insert(int(New_Col_position) - 1, column_name, column_data)
        # data_frame.insert()

        return data_frame

    '''
    function Name: get more complex data from robot files
    Purpose: Load data from a file into a NumPy array. Then return it as csv data.
    '''
    def get_Data_Complex(self):

        data_file = input("Enter file name: ")
        with open(f'/Users/100655277/Desktop/Python/MachineLearning/Lab1/{data_file}.data', "r") as file:
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
                        matrix[rowCounter, columnCounter]  = round(int(rows[columnCounter]))
                        columnCounter = columnCounter + 1
                    
                    rowCounter = rowCounter + 1
            else:
                if current_row:
                    data.append(current_row)
                current_row = []


        print(matrix)
        np.savetxt("output_matrix.csv", matrix, delimiter=",")
        # df = pd.read_csv("output_matrix.csv")
        data_file = "output_matrix"
        num_columns = len(pd.read_csv(f'/Users/100655277/Desktop/Python/{data_file}.csv').columns)

        #create column names
        column_names = [f'column_{i}' for i in range(1, num_columns+1)]
        #read csv file into a data frame, and name columns
        df = pd.read_csv(f'/Users/100655277/Desktop/Python/{data_file}.csv', names = column_names)

        return df


    """
    Split a dataset into training and testing sets.

    Args:
        dataset (pandas DataFrame): The dataset to be split.
        test_size (float): The proportion of the dataset to include in the testing set.
        random_state (int): Seed for the random number generator (optional).

    Returns:
        tuple: A tuple containing the training set and testing set DataFrames.
    """
    
    def split_dataset(self, dataset, test_size=0.2, random_state=None):
        
        train_set, test_set = train_test_split(dataset, test_size=test_size, random_state=random_state)
        # print(train_set)
        return train_set, test_set
    
        