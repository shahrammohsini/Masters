import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
import pandas as pd
import joblib
from sklearn.preprocessing import StandardScaler

# Create a StandardScaler instance
scaler = StandardScaler() #will be used to normalize

# Step 1: Load your dataset
data = pd.read_csv(r'C:\Users\100655277\Documents\GitHub\Masters\windows\Python\MachineLearning\Machine_Learning_PID_Project\Bike_Training_Data\pid_results.csv', na_values='Nan')
data = data.dropna()
print(data)

# Step 2: Prepare the dataset
X = data[['Rise Time (s)', 'Settling Time (s)', 'Percentage Overshoot (%)', 'Steady-State Error (rad)']]
y = data[['P', 'I', 'D']]

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Step 3: Train the linear regression model
model = LinearRegression()
model.fit(y_train, X_train)  # Invert the inputs and outputs

joblib.dump(model, 'Bike_PID_to_Parmters_LinearRegrssion_Model.pkl')  # Save the model for later use

# Step 4: Evaluate the model
y_pred = model.predict(y_test)
mse = mean_squared_error(X_test, y_pred)
r2 = r2_score(X_test, y_pred)
print(f"Mean Squared Error: {mse}")
print(f"R-squared: {r2}")

# Step 5: Use the trained model to predict new_rise_time, new_settling_time, new_overshoot, and new_steady_state_error
new_P = 150  # Replace with your desired values
new_I = 11
new_D = 0.8
new_data = np.array([[new_P, new_I, new_D]])
predicted_values = model.predict(new_data)
new_rise_time, new_settling_time, new_overshoot, new_steady_state_error = predicted_values[0]

print(f"Predicted Rise Time: {new_rise_time}")
print(f"Predicted Settling Time: {new_settling_time}")
print(f"Predicted Percentage Overshoot: {new_overshoot}")
print(f"Predicted Steady-State Error: {new_steady_state_error}")

# loaded_model = joblib.load('Bike_PID_to_Parmters_LinearRegrssion_Model.pkl')  # Load the model

# predicted_values = loaded_model.predict(new_data)
# new_rise_time, new_settling_time, new_overshoot, new_steady_state_error = predicted_values[0]

# print(f"Predicted Rise Time: {new_rise_time}")
# print(f"Predicted Settling Time: {new_settling_time}")
# print(f"Predicted Percentage Overshoot: {new_overshoot}")
# print(f"Predicted Steady-State Error: {new_steady_state_error}")
