import numpy as np
from sklearn.metrics import mean_squared_error
from scipy.optimize import minimize
import joblib #lets us save the trained model
import warnings
warnings.filterwarnings("ignore", category=UserWarning)



# Define a function that calculates the objective to minimize
def objective(params):
    P, I, D = params
    loaded_model = joblib.load('Bike_PID_to_Parmters_LinearRegrssion_Model.pkl')
    # Use your trained inverse model to predict new_rise_time, new_settling_time, new_overshoot, new_steady_state_error
    predicted_values = loaded_model.predict(np.array([[P, I, D]]))
    new_rise_time, new_settling_time, new_overshoot, new_steady_state_error = predicted_values[0]
    # Define a cost function to minimize (e.g., you can sum these values)
    cost = new_rise_time + new_settling_time + new_overshoot + new_steady_state_error
    return cost

# Set up the grid search space for P, I, D
param_grid = {
    'P': np.linspace(0, 200, 200),  # Adjust the range as needed
    'I': np.linspace(0, 100, 100),  # Adjust the range as needed
    'D': np.linspace(0, 10, 30),  # Adjust the range as needed
}

# Initialize the best values and minimum cost
best_params = None
min_cost = float('inf')

# Loop through the grid search space
for P in param_grid['P']:
    for I in param_grid['I']:
        for D in param_grid['D']:
            print("PID - ", P, " - ", I, " - ", D)
            params = [P, I, D]
            cost = objective(params)
            if cost < min_cost:
                min_cost = cost
                best_params = params

best_P, best_I, best_D = best_params
print(f"Best P: {best_P}")
print(f"Best I: {best_I}")
print(f"Best D: {best_D}")
print(f"Minimum Cost: {min_cost}")
