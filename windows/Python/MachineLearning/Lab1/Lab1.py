import pandas as pd
import sys
sys.path.append('MachineLearning/Functions')
from sklearn.model_selection import train_test_split
import functions as fn
import time

# Record the start time
start_time = time.time()



if __name__ == "__main__":

            print("Program start")
            setup = fn.Setup()

            # data_frame = setup.get_Data_Complex()
            data_frame = setup.getData()
            data_frame = setup.moveColumn(data_frame = data_frame)
            print(data_frame)
            train_set, test_set = setup.split_dataset(dataset = data_frame, test_size=0.2, random_state=None)
            print("Training set: ", train_set)
            print("test set: ", test_set)


end_time = time.time()
# Calculate the execution time
execution_time = end_time - start_time

print(f"Execution time: {execution_time} seconds")