# from vpython import *
import math
import csv


#get rise time
def calculate_rise_time(angle_data, desired_angle, dt):
    """
    Calculate the rise time of an angle signal from a starting value to a desired value.

    Args:
    angle_data (list): A list of angle values over time.
    desired_angle (float): The desired angle value.

    Returns:
    float: The rise time in seconds, or None if the desired angle is not reached.
    """
    tolerance = 0.1 #The rise time should be the time it takes to get 90% of the desired angle
    rise_time = None
    for i, angle in enumerate(angle_data):
        if abs(angle - desired_angle) <= abs(tolerance * desired_angle):
            if rise_time is None:
                rise_time = i * dt  # Start measuring time when the angle is within tolerance
        elif rise_time is not None:
            return rise_time  # Return the rise time when the angle goes outside of tolerance

    return "Nan"  # If the desired angle within tolerance was not reached




def calculate_settling_time(dt, angle_data, desired_angle, threshold_percentage):
    """
    Calculate the settling time of an angle signal within a given threshold percentage of the desired angle.

    Args:
    angle_data (list): A list of angle values over time in radians.
    desired_angle (float): The desired angle value in radians.
    threshold_percentage (float): The percentage threshold within which the signal should settle. Default is 5%.

    Returns:
    float: The settling time in seconds, or None if not settled within the threshold.
    """
    threshold = threshold_percentage * abs(desired_angle)
    settling_time = "Nan"
    within_threshold = 0
    
    #If the angle is withink the desired threshold for 10 counts we consider that good enough
    for i, angle in enumerate(angle_data):
        if abs(angle - desired_angle) <= threshold:
            within_threshold += 1
        else:
            within_threshold = 0  # Reset the count if the signal goes outside the threshold
        if within_threshold >= 10:  # Adjust the count based on your settling criteria
            settling_time = i * dt  # Assuming 'dt' is the time step used in your simulation
            break
    
    return settling_time





def calculate_percentage_overshoot(angle_data, desired_angle):
    """
    Calculate the percentage overshoot of an angle signal.

    Args:
    angle_data (list): A list of angle values over time in radians.
    desired_angle (float): The desired angle value in radians.

    Returns:
    float: The percentage overshoot as a positive percentage of the desired angle, or None if no overshoot occurred.
    """
    max_overshoot = 0
    overshoot_started = False

    for angle in angle_data:
        if angle >= desired_angle:
            overshoot_started = True
            deviation = angle - desired_angle
            if deviation > max_overshoot:
                max_overshoot = deviation
        elif overshoot_started:
            break  # Stop when overshoot ends

    if max_overshoot <= 0:
        return "Nan"  # No overshoot occurred
    percentage_overshoot = (max_overshoot / abs(desired_angle)) * 100
    return percentage_overshoot



def calculate_steady_state_error(angle_data, desired_angle):
    """
    Calculate the steady-state error of an angle signal.

    Args:
    angle_data (list): A list of angle values over time in radians.
    desired_angle (float): The desired angle value in radians.

    Returns:
    float: The steady-state error as the absolute difference between the final value and the desired angle.
    """
    final_value = angle_data[-1]  # Get the final (last) value in the response data

    steady_state_error = abs(final_value - desired_angle)
    return steady_state_error


def Model(kp, ki, kd, desired_angle, initial_angle, dt):
    #Parameters
    desired_angle = desired_angle  #desired control angle
    thetaR = initial_angle # angle of rod  (Initial rod angle)
    thetaR = -math.radians(thetaR)
    radius_f1 = 0.05
    radius_f2 = 0.06
    radius_f = 0.06
    g = 9.8
    L = 0.133 # Length of Rod 0.17
    Xf = L*math.sin(thetaR) # x pos of flywheel
    Yf = L*math.cos(thetaR) # y pos of flywheel
    mR = 0.027 # mass of rod in kg           0.033kg
    mF = 0.06 # mass of flywheels in kg     0.588kg
    mm = 0.11 #mass of motor
    M = 0.5*mR + mF
    thetaF = 0 #sping angle of flywheel
    thetaF = -math.radians(thetaF)
    Torque_M = 0 # input torque of motor
    t = 0
    dt = dt
    i = 0
    k =  0.006 #0.00157  #coeffecient of friction
    thetadotR = 0
    thetadotF = 0
    IR = (1/3)*mR*L*L  #Moment of interti of rod
    IF = 0.5*mF*((radius_f1*radius_f1) + (radius_f2*radius_f2))  #Moment of intertia of flywheel
    IL = IR + mF*L*L #For conviniance let IL = IR + mF*L*L
    #Controller
    error = 0
    kp = kp #1000 # 1.3  300
    kd = kd#10 #prevent overshooting   0.2
    ki = ki #40 #eleminates steady state error  0.2
    previous_integral = 0
    previous_error = 0
    stall_torque = 1.1 # Nm

    #for the motor
    Rm = 2.5 #internal resitance of motor
    kt = 0.33#0.3568 #torque constant of motor
    Lm = 0.9 #0.499 #internal inductance of motor


    angle_data = []


    while t < 20:
        i = i + 1
        # Text_Time.text = f'Time: {i/100} Sec'
        
        # rate(100)

        #Control
        error = thetaR + math.radians(desired_angle)
        # print(math.degrees(error))
        # if math.degrees(error) < 0.00001: print("true")
        integral = previous_integral + error*dt
        P = kp * error
        I = ki * integral
        D = kd * ((error - previous_error)/dt)
        # Torque_M = P + I + D
        PWM = P + I + D

        # Clamping anti-windup on integral to prevent windup
        if PWM > 255 and error > 0:  # if the output is maxed out and error is still positive
            integral = previous_integral  # don't accumulate
        elif PWM < -255 and error < 0:  # if the output is at its minimum and error is still negative
            integral = previous_integral  # don't accumulate
        
    
        V = 12*(PWM/255) #input voltage


       
        
        # print("Integral: ", integral)

        #Limits (saturation)
        if V > 12:
            V = 12
        elif V < -12:
            V = -12


        Torque_M = kt*V*math.exp((-Rm/Lm))
        # print("Torque_M", Torque_M)

        # print("Torque: ", Torque_M)



        #Simulation
        #thetaddotR = ((0.5*mR + mF)*g*L*thetaR - Torque_M) / ((1/3)*mR*L*L + mF*L*L)  #Linear equation
        #thetaddotR = ((M*g*L*sin(thetaR)) - Torque_M) / IL
        thetaddotR = ((0.5*mR + mF)*g*L*math.sin(thetaR) - Torque_M) / ((1/3)*mR*L*L + mF*L*L)  #without friction
        # thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M - (thetadotR*k)) / ((1/3)*mR*L*L + mF*L*L + mm*L*L) #with friction
        thetadotR = thetadotR + thetaddotR*dt
        thetaR = thetaR + thetadotR*dt
        angle_data.append(thetaR)


        # thetaddotF = Torque_M/IF
        # thetadotF = thetadotF + thetaddotF*dt
        # thetaF = thetaF + thetadotF*dt
        # thetadotw = thetadotF - thetadotR



        previous_integral = integral
        previous_error = error
        t = t + dt
        
    return angle_data


    
    # print("outside the loop")

    # rise_time = calculate_rise_time(angle_data, math.radians(desired_angle))
    # print("rise time: " , rise_time)

    # settling_time = calculate_settling_time(angle_data, math.radians(desired_angle), threshold_percentage=0.05)
    # print("settling time: " , settling_time)

    # percentage_overshoot = calculate_percentage_overshoot(angle_data, math.radians(desired_angle))
    # print("percentage_overshoot: " , percentage_overshoot)

    # steady_state_error = calculate_steady_state_error(angle_data, math.radians(desired_angle))
    # print("steady_state_error: " , steady_state_error)





if __name__ == "__main__":
    # kp = 150
    # ki = 80
    # kd = 2

    # P_values = range(500)  # P from 0 to 100
    # I_values = range(200)  # I from 0 to 100
    # D_values = [d / 100 for d in range(20*100)] # D from 0.00 to 20.00 with a step of 0.01
    P_values = range(200)  # P from 0 to 300
    I_values = range(200)  # I from 0 to 200
    # D_values = [d / 100 for d in range(200)] # D from 0.00 to 20.00 with a step of 0.01
    D_values = [0.2 * i for i in range(12)]  # Creates a list [0, 0.2, 0.4, ... 19.8, 20.0]
    D_values = [round(value, 2) for value in D_values] #rounding to 2 decimal spaces
    
    desired_angle = 0.01
    initial_angle = 7
    dt = 0.01

    print("D_values: ", (D_values))
    # angle_data = []
    # angle_data = Model(150, 80, 5, desired_angle, initial_angle, dt)
    # settling_time = calculate_settling_time(dt, angle_data, math.radians(desired_angle), threshold_percentage=10)
    # print("settling time: " , settling_time)

    # Create a CSV file to save the results
    with open(r'C:\Users\100655277\Documents\GitHub\Masters\windows\Python\MachineLearning\Machine_Learning_PID_Project\Bike_Training_Data\pid_results_5.csv', 'w', newline='') as csvfile:
        fieldnames = ['P', 'I', 'D', 'Rise Time (s)', 'Settling Time (s)', 'Percentage Overshoot (%)', 'Steady-State Error (rad)']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()  # Write the header row

        for P in P_values:
            for I in I_values:
                for D in D_values:
                    print("PID: ", P , "-", I , "-", D)
                    angle_data = []
                    angle_data = Model(P, I, D, desired_angle, initial_angle, dt)

                    # print("angle_data: " , angle_data)

                    rise_time = calculate_rise_time(angle_data, math.radians(desired_angle), dt)
                    # print("rise time: " , rise_time)

                    settling_time = calculate_settling_time(dt, angle_data, math.radians(desired_angle), threshold_percentage=10) #this will consider anything withibn 0 to 0.1 deg settled
                    # print("settling time: " , settling_time)

                    percentage_overshoot = calculate_percentage_overshoot(angle_data, math.radians(desired_angle))
                    # print("percentage_overshoot: " , percentage_overshoot)

                    steady_state_error = calculate_steady_state_error(angle_data, math.radians(desired_angle))
                    # print("steady_state_error: " , steady_state_error)

                    # Write the results to the CSV file
                    writer.writerow({'P': P, 'I': I, 'D': D, 'Rise Time (s)': rise_time, 'Settling Time (s)': settling_time,
                                        'Percentage Overshoot (%)': percentage_overshoot, 'Steady-State Error (rad)': steady_state_error})





