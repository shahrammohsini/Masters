"""This file holds all the functions required for MPC motor controll
"""
import RPi.GPIO as GPIO
import time
import json
import copy
import numpy as np
import math


"""
Purpose: Drive a motor using PWM signals
Variables: enA, in1, in2 (Motor driver pins)
How to use: Instantiate the Motor class with the pins as arguments and then call the drive or stop methods as needed.
Example: motor = Motor(enA=26, in1=5, in2=6)
         motor.drive(pwmOutput=255)
"""
class Motor:
    #Using Singlton Design patter to ensure the same class doesn't have more than one initialization which causes errors
# Dictionary to hold instances of Motor class (Singleton pattern)
    _instances = {}

    def __new__(cls, *args, **kwargs):
        # Check if an instance already exists for this class
        if cls not in cls._instances:
            # If not, create a new instance
            instance = super(Motor, cls).__new__(cls)
            # Call the initialization method for the newly created instance
            instance.init(*args, **kwargs)
            # Store the instance in the _instances dictionary
            cls._instances[cls] = instance
        
        # Return the existing instance (or the newly created one)
        return cls._instances[cls]
    
    #End of singleton design pattern
    def init(self, enA, in1, in2):
        self.enA = enA
        self.in1 = in1
        self.in2 = in2
        self.setup()
        
    def setup(self):
        GPIO.setup(self.enA, GPIO.OUT)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm = GPIO.PWM(self.enA, 1000)
        self.pwm.start(0)
        
    def drive(self, pwmOutput):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(pwmOutput / 255.0 * 100)
        
    def stop(self):
        self.pwm.stop()


"""
Purpose: Read encoder data
variables: ENCA, ENCB (Both are the pins the encoder is connected to)
How to use: call the Encoder class with inputs as ENCA and ENCB and read posi variable. 
Example:
        encoder = Encoder(ENCA=7, ENCB=8)
        encoder.posi
"""
class Encoder:
    #Using Singlton Design patter to ensure the same class doesn't have more than one initialization which causes errors
    # Dictionary to hold instances of Encoder class (Singleton pattern)
    _instances = {}

    def __new__(cls, *args, **kwargs):
        # Check if an instance already exists for this class
        if cls not in cls._instances:
            # If not, create a new instance
            instance = super(Encoder, cls).__new__(cls)
            # Call the initialization method for the newly created instance
            instance.init(*args, **kwargs)
            # Store the instance in the _instances dictionary
            cls._instances[cls] = instance
        
        # Return the existing instance (or the newly created one)
        return cls._instances[cls]
    #End of singleton design pattern

    def init(self, ENCA, ENCB):
        self.ENCA = ENCA
        self.ENCB = ENCB
        self.posi = 0
        self.setup()
        
    def setup(self):
        GPIO.setup(self.ENCA, GPIO.IN)
        GPIO.setup(self.ENCB, GPIO.IN)
        GPIO.add_event_detect(self.ENCA, GPIO.RISING, callback=self.read)
    
    def read(self, channel):
        b = GPIO.input(self.ENCB)
        if b:
            self.posi += 1
        else:
            self.posi -= 1




"""
Purpose: Calculate velocity based on encoder readings
Variables: CPR (Counts per revolution of encoder)
How to use: Instantiate the VelocityCalculator class with CPR as argument and then call the calculate method with current position.
Example: velocity_calculator = VelocityCalculator(CPR=11)
         velocity = velocity_calculator.calculate(encoder.posi)
"""
class VelocityCalculator:
    def __init__(self):
        self.CPR = 11 #number of encoder ticks in one revolution
        self.prevTime = 0
        self.prevPos = 0
    
    def calculate(self, currentPos):
        currentTime = time.time()
        timeDiff = currentTime - self.prevTime
        time.sleep(0.8)
        
        if timeDiff > 0:
            velocity = ((currentPos - self.prevPos) / self.CPR) / timeDiff
            self.prevPos = currentPos
            self.prevTime = currentTime
            return velocity * 60 #Return velocity in RPM
        





"""
Purpose: Create the Dynamic Matrix
Variables: voltage, PWM_val, loop_time
How to use: Instantiate the DynamicMatrix class as argument and then call the any of its functions to use it
Example: dynamic_matrix = f.DynamicMatrix()  
         dynamic_matrix.data_writer(voltage=6, pwm_val = 150, loop_time = 20) //collect open loop data for 6v, 150 pwm, and 20 sec into a json file
"""
class DynamicMatrix:
    # def __init__(self):
    #     self.voltage = voltage
    #     self.PWM_val = PWM_val
    #     self.loop_time = loop_time

    def write_to_array(self,pwm_val, loop_time):  #Drive the motor and collect data into an array
        GPIO.setmode(GPIO.BCM)
        data_points = []
        startTime = time.time()
        motor = Motor(enA=26, in1=5, in2=6)
        encoder = Encoder(ENCA=7, ENCB=8)
        velocity_calculator = VelocityCalculator()
        try:
            while (time.time() - startTime) < loop_time:
                velocity = abs(velocity_calculator.calculate(encoder.posi))
                time_t = time.time() - startTime
                # Print data
                print(f"Time:{round(time_t,2)}", f"Velocity:{round(velocity, 2)}")
                #Enter Data into array
                data_points.append({"Time":round(time_t,2), "Velocity":round(velocity,2)})
                # Drive the motor
                motor.drive(pwmOutput = pwm_val) #set motor speed here
                
                time.sleep(1.5)
        
        except KeyboardInterrupt:
            motor.stop()
            GPIO.cleanup()

        return data_points

    #Enter Data into json file
    def data_writer(self,voltage, pwm_val, loop_time):
        data_points = []
        data_points = self.write_to_array(pwm_val, loop_time)
        try:
        
            with open(f"/home/sham/Desktop/pythonFiles/motor_Control/Data/{voltage}_volts.json", mode="w") as file:
                json.dump(data_points, file, indent=1)
            file.close
        except Exception as e:
            print(f"Error while writing data to the file: {e}")


    
    #Create the A matrix
    def A_Matrix(self, voltage, nu):
        with open(f"/home/sham/Desktop/pythonFiles/motor_Control/Data/{voltage}_volts.json", mode='r') as file:
            data_points = json.load(file)
        velocity_Array = [data_point["Velocity"] for data_point in data_points]
        # Divide every value in velocity_Array by voltage to normalize
        velocity_Array = [round(v / voltage,2) for v in velocity_Array]

        # velocity_Array = [1,2,3,4,5,6,7,8,9]
        nu = 3
        r = len(velocity_Array) #get number of rows
        Mod_Array = copy.deepcopy(velocity_Array)
        for i in range(nu-1):
            Mod_Array.insert(0,0)  # This inserts N-1 zeros into array Mod_Array


        C = nu #get number of columns
        
        m = [] #create matrix (array of arrays)
        #Create the A matrix
        for i in range (r):
            E = []
            for j in range(C):
                v = Mod_Array[(i - 1 + (nu - j))]  #formula to fill dynamic matrix appropriately.
                E.append(v)
            m.append(E)

        return(m)



class Maths: #Used to map voltage to pwm
    def map(self, input, inMin, inMax, outMin, outMax):
        return outMin + (float(input - inMin) / float(inMax - inMin) * (outMax
                        - outMin))


#MPC controlelr

class MPC_controller:
    def __init__(self):
        self.encoder = Encoder(ENCA=7, ENCB=8)
        self.DM = DynamicMatrix()
        self.velocity = VelocityCalculator()
        self.motor = Motor(enA=26, in1=5, in2=6)
        self.maths = Maths()

    


    def controller(self, voltage, nu):

        #Constants
        # N = int(3/0.01 + 1)  #(prediction horizon) number of data points in matrix. ran for 3 sec and collected 100 data points ever sec. the plus 1 is bc of the way I designed my loop
        A = self.DM.A_Matrix(voltage , nu) #get the A matrix
        N = len(A)
        nu = 3 #control horizon
        u = np.zeros(N) #Voltage (Step input)for model calculation
        Y = 0
        PHI = np.zeros(nu) #To correct model inaccuracy
        LAMBDA = 1.01 #penalty factor
        y_mes = 0  #Measured position
        #error = np.zeros((3, 1)) # r_desired_vel - y_hat  #error between desired velocity and predicted velocity
        error = 0
        r_desired_vel = 1400 #RPM   Desired velocity setpoint
        delta_u = np.zeros(N) #Optimized change in input (u) to minimize error 
        u_prev = np.zeros(N) # input from previous iteration
        I_Matrix = np.identity(nu) # create an identity matrix of size 3 by 3
        A_T = np.transpose(A)  #Transpose of matrix A
        y_hat = delta_u.dot(A) #Predicted position
        


        while True:

            Y = round(self.velocity.calculate(self.encoder.posi),2) #get the motor velocity from encoder
            print(Y)
            ym = Y
            #MPC
            PHI = ym - y_hat #difference between calculated velocity (y_hat) and measured velocity (ym) to see if the model is inaccurate
            y_hat = y_hat + PHI #correct model inacuracy
            error = r_desired_vel - y_hat
            print(error)
            delta_u = error.dot(np.linalg.inv(A_T.dot(A) - (LAMBDA * I_Matrix)).dot(A_T)) #optimize input required to remove error
            print(delta_u)
            u = u_prev + delta_u # update input with optimized change
            # print(np.linalg.inv(A_T.dot(A) - (LAMBDA * I_Matrix)))
            # print((A_T))
            # print(u)
            # Add limit control on input U
            if u[0] > 12:
                u[0] = 12

            elif u[0] < -12:
                u[0] = -12
        
            delta_u = u - u_prev #This recalculates delta_u so that it includes the limits added for the next step where we calculate y_hat
            #print(delta_u.size)
            
            y_hat = y_hat + delta_u.dot(A) #update predicted output (note the delta_u used includes the limits added)
            y_hat[:-1] = y_hat[1:] #This line only moves the values of y_hat one to the right
            #ex: y_hat before = [1,2,3] y_hat after = [2,3,3]. Note the last value is repeated. All this does is predict the next step
            print("u", u[1])
            pwminput = self.maths.map(u[1], 0, 12, 0, 255)
            print(f"pwm input = {u[1]}")
            self.motor.drive(pwminput)
            #update values
            u_prev = u





