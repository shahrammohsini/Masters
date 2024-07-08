import matplotlib.pyplot as plt

t = 0
dt = 0.01 # If this step size is too large the model becomes very innacurate
voltage = 12
previous_velocity = 0
current_Velocity = 0

time = []
velocity = []

while t < 10:
    time.append(t)
    velocity.append(current_Velocity)
    print(current_Velocity)

    current_Velocity = previous_velocity + dt*(-22.37*previous_velocity + 506.7*voltage)

    previous_velocity = current_Velocity

    t = t + dt

plt.plot(time, velocity)
plt.show()