import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
import pandas as pd
import joblib #lets us save the trained model
from sklearn.preprocessing import StandardScaler

# Create a StandardScaler instance
scaler = StandardScaler()


# Step 1: Load your dataset
# Assuming you have your data in a CSV file named 'pid_data.csv'
data = pd.read_csv(r'C:\Users\100655277\Documents\GitHub\Masters\windows\Python\MachineLearning\Machine_Learning_PID_Project\Bike_Training_Data\pid_results_5.csv', na_values='Nan' )
# Remove rows with missing values (NaN)
data = data.dropna()
print(data)

# Step 2: Prepare the dataset
# Extract the independent variables (X) and dependent variables (y)
X = data[['Rise Time (s)', 'Settling Time (s)', 'Percentage Overshoot (%)', 'Steady-State Error (rad)']]
y = data[['P', 'I', 'D']]


X = scaler.fit_transform(X)  # Normalize all the Features


X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Step 3: Train the linear regression model
model = LinearRegression()
model.fit(X_train, y_train)

joblib.dump(model, 'Bike_PID_linear_regression_model.pkl') #save the model for later use

# Step 4: Evaluate the model
y_pred = model.predict(X_test)
mse = mean_squared_error(y_test, y_pred)
r2 = r2_score(y_test, y_pred)
print(f"Mean Squared Error: {mse}")
print(f"R-squared: {r2}")


new_rise_time = 0
new_settling_time = 0
new_overshoot = 0
new_steady_state_error = 0
# Step 5: Use the trained model to predict P, I, and D values
new_data = np.array([[new_rise_time, new_settling_time, new_overshoot, new_steady_state_error]])
predicted_values = model.predict(new_data)
# P_predicted, I_predicted, D_predicted = predicted_values[0], predicted_values[1], predicted_values[2]
# print(f"Predicted P: {P_predicted}")
# print(f"Predicted I: {I_predicted}")
# print(f"Predicted D: {D_predicted}")
print("Predicted P I D Values: ", predicted_values)

# loaded_model = joblib.load('Bike_PID_linear_regression_model.pkl') #this is how we re loat the data

# predicted_values = loaded_model.predict(new_data)
# print("Predicted_values: ", predicted_values)


