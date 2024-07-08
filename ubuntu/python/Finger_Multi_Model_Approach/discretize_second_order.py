from scipy import signal

# Define the continuous-time transfer function
num = [9.26, 448.8]
den = [1, 7.326, 37.82]

# Define the sampling time
T = 0.03  #  sampling time of 0.03 seconds

# Discretize using bilinear transform
num_d, den_d = signal.bilinear(num, den, fs=1/T)

print("Discrete numerator coefficients:", num_d)
print("Discrete denominator coefficients:", den_d)