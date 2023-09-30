"""This file holds all the functions required for MPC motor controll
"""
import RPi.GPIO as GPIO
import time
import json
import copy
import numpy as np
import math
from vpython import *
# import threading


"""
Purpose: Drive a motor using PWM signals
Variables: pwm_pin, brake_pin, direction_pin (Motor driver pins)
How to use: call the drive or stop methods as needed.
Example: motor = Motor()
         motor.drive(pwmOutput= 255, direction= "ccw")
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
        
    def drive(self, pwmOutput, direction):
        if direction == "ccw":
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
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
        # self.prevT = time.time()
        # self.velocity = 0

        self.setup()
        
    def setup(self):
        GPIO.setup(self.ENCA, GPIO.IN)
        GPIO.setup(self.ENCB, GPIO.IN)
        GPIO.add_event_detect(self.ENCA, GPIO.RISING, callback=self.read)
    
    def read(self, channel):
        b = GPIO.input(self.ENCB)
        if b:
            self.posi += 1
            # print("pos: ",self.posi)

            # METHOD TWO FOR VELOCITY CALCULATION
            # currT = time.time()
            # deltaT = self.prevT - currT
            # self.velocity = ((1 / deltaT)/11)*60
            # self.prevT = currT
        else:
            self.posi -= 1
            # print("pos: ",self.posi)

            # METHOD TWO FOR VELOCITY CALCULATION
            # currT = time.time()
            # deltaT = self.prevT - currT
            # self.velocity = 1 / deltaT
            # self.prevT = currT
            
        
        




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
        # time.sleep(0.1)
        currentTime = time.time()
        timeDiff = currentTime - self.prevTime
        
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

    def write_to_array(self,pwm_val, loop_time, time_step):  #Drive the motor and collect data into an array
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
                motor.drive(pwmOutput = pwm_val, direction = "cw") #set motor speed here
                
                time.sleep(time_step)
        
        except KeyboardInterrupt:
            motor.stop()
            GPIO.cleanup()

        return data_points

    #Enter Data into json file
    def data_writer(self,voltage, pwm_val, loop_time, time_step): #Time step is the time it takes to get to 62.3% of your steady state
        data_points = []
        data_points = self.write_to_array(pwm_val, loop_time, time_step)
        try:
        
            with open(f"/home/sham/Desktop/github/Masters/raspbian/pythonFiles/motor_Control/Data/{voltage}_volts.json", mode="w") as file:
                json.dump(data_points, file, indent=1)
            file.close
        except Exception as e:
            print(f"Error while writing data to the file: {e}")
        return data_points

    
    #Create the A matrix
    def A_Matrix(self, voltage, nu, normalize_val):
        with open(f"/home/sham/Desktop/github/Masters/raspbian/pythonFiles/motor_Control/Data/{voltage}_volts.json", mode='r') as file:
            data_points = json.load(file)
        velocity_Array = [data_point["Velocity"] for data_point in data_points]
        # Divide every value in velocity_Array by voltage to normalize
        velocity_Array = [(v / normalize_val) for v in velocity_Array] #Must divide by the voltage applie to normalize the vallue.

        # # velocity_Array = [1,2,3,4,5,6,7,8,9]
        # r = len(velocity_Array) #get number of rows
        # Mod_Array = copy.deepcopy(velocity_Array)
        # for i in range(nu-1):
        #     Mod_Array.insert(0,0)  # This inserts N-1 zeros into array Mod_Array


        # C = nu #get number of columns
        
        # m = [] #create matrix (array of arrays)
        # #Create the A matrix
        # for i in range (r):
        #     E = []
        #     for j in range(C):
        #         v = Mod_Array[(i - 1 + (nu - j))]  #formula to fill dynamic matrix appropriately.
        #         E.append(v)
        #     m.append(E)

        #Meaghan's method
        N = len(velocity_Array) #get number of rows
        Dynamic_Matrix = np.zeros([N,nu])
        #Note this method is set up so that the first row of col 2 and 3 are not set also the sec row of col 3 is not set which means they stay 0
        for j in range(0,nu):
            for i in range(0, N-j): #N-j means it won't run more that the size of the matrix for col 2 and 3 where row 1 (col2) and row 1 and 2 (col3) are 0
                Dynamic_Matrix[i+j, j] = velocity_Array[i]


        return(Dynamic_Matrix)



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
        self.v = Visual()

    def controller(self, voltage, nu, desired_vel, loop_time,normalize_val, time_Step, LAMBDA):

        #Constants
        # N = int(3/0.01 + 1)  #(prediction horizon) number of data points in matrix. ran for 3 sec and collected 100 data points ever sec. the plus 1 is bc of the way I designed my loop
        A = self.DM.A_Matrix(voltage, nu, normalize_val) #get the A matrix
        N = len(A)
        u = np.zeros(nu) #Voltage (Step input)for model calculation
        Y = 0
        PHI = np.zeros(nu) #To correct model inaccuracy
        # LAMBDA = 2.01 # 0.6 0.5 and 0.55 penalty factor
        # y_mes = 0  #Measured position
        #error = np.zeros((3, 1)) # r_desired_vel - y_hat  #error between desired velocity and predicted velocity
        # error = np.zeros()
        setpoint = desired_vel #RPM   Desired velocity setpoint
        r_desired_vel = setpoint*np.ones(N)
        u_prev = np.zeros(nu) # input from previous iteration
        I_Matrix = np.identity(nu) # create an identity matrix of size 3 by 3
        A_T = np.transpose(A)  #Transpose of matrix A
        y_hat = np.zeros(N) #Predicted position
        # dt = 0.02 #0.65 and 0.7
        timer = 0
        data_points = []
        #Du calculations. A is the dynamixc matrix
        LambdaI = LAMBDA * I_Matrix
        print(np.shape(LambdaI))
        ATA = A_T.dot(A)
        ATA_LambdaI = ATA - LambdaI
        ATA_LambdaI_Inv = np.linalg.inv(ATA_LambdaI)
        Du = ATA_LambdaI_Inv.dot(A_T)


        startTime = time.time()

        while timer < loop_time:
            timer = time.time() - startTime
            time.sleep(time_Step)

            enVelocity = self.velocity.calculate(self.encoder.posi) #get the motor velocity from encoder

            # if abs(enVelocity) > 4000: #This is in the hopes of filtering out the noise
            #     enVelocity = 0
            # # Y = 100
            # print("velocity: ",enVelocity)
            ym = enVelocity
            #MPC
            PHI = ym - y_hat[0] #difference between calculated velocity (y_hat) and measured velocity (ym) to see if the model is inaccurate

            # print("PHI: ",PHI)
            
            # print("y_hat_a: ",y_hat)
            y_hat = y_hat + PHI #correct model inacuracy
            # print("y_hat: ",y_hat)
            error = r_desired_vel - y_hat
            # delta_u = error.dot(np.linalg.inv(A_T.dot(A) - (LAMBDA * I_Matrix)).dot(A_T)) #optimize input required to remove error
            delta_u = np.dot(Du,error)
            u = u_prev + delta_u # update input with optimized change
            if u[0] > 255:
                u[0] = 255

            elif u[0] < -255:
                u[0] = -255
        
            delta_u = u - u_prev #This recalculates delta_u so that it includes the limits added for the next step where we calculate y_hat
            #print(delta_u.size)
            # y_hat = y_hat + delta_u.dot(A) #update predicted output (note the delta_u used includes the limits added)
            y_hat = y_hat + A[:,0]*delta_u[0] #update predicted output (note the delta_u used includes the limits added). A[:,0] all rows of first col

            # print("old yhat: ", y_hat)
            y_hat[:-1] = y_hat[1:] #This line only moves the values of y_hat one to the right
            # print("new Y_hat: ",y_hat)
            #ex: y_hat before = [1,2,3] y_hat after = [2,3,3]. Note the last value is repeated. All this does is use the next predicted step
            # print("u", u[1])
            if u[0] > 0:
                # pwminput = self.maths.map(u[0], 0, 5000, 0, 255)
                self.motor.drive(abs(u[0]),"cw")
            else:
                # pwminput = self.maths.map(u[0], 0, -5000, 0, 255)
                self.motor.drive(abs(u[0]),"ccw")
            #print(f"pwm input = {pwminput}")
            #update values
            u_prev = u

            #graph
            self.v.graph(velocity = enVelocity, time = timer, error = r_desired_vel, input = u[0] )

            #Save data
            data_points.append({"Time":timer, "Velocity":enVelocity})
            
        return data_points








class PID_controller:
    def __init__(self):
        self.encoder = Encoder(ENCA=7, ENCB=8)
        self.DM = DynamicMatrix()
        self.velocity = VelocityCalculator()
        self.motor = Motor(enA=26, in1=5, in2=6)
        self.maths = Maths()
        self.v = Visual()


    def controller(self, desired_vel, loop_time, kp, ki, kd):

        #Constants
        dt = 0.15
        error = 0
        kp = kp  #1.5
        kd = kd  #0.05
        ki = ki  #0.9
        integral = 0
        prev_integral = 0
        prev_error = 0
        PID_current_Timer = 0
        PID_Prev_Timer = 0
        P = 0
        I = 0
        D = 0
        u = 0 #input
        max_vel = 6340
        data_points = []
        timer = 0

        
        startTime = time.time()
        while timer < loop_time:

            timer = time.time() - startTime
            time.sleep(dt)

            enVelocity = self.velocity.calculate(self.encoder.posi) #get the motor velocity from encoder
            print("velocity: ", enVelocity)
            print("integral: ", I)
            
            error = desired_vel - enVelocity
            integral = prev_integral + error*dt
            P = kp*error 
            I = (ki*integral)
            D = kd* ((error-prev_error)/dt)
            u = P + I + D

            # Clamping anti-windup on integral to prevent windup
            if u > max_vel and error > 0:  # if the output is maxed out and error is still positive
                integral = prev_integral  # don't accumulate
            elif u < -max_vel and error < 0:  # if the output is at its minimum and error is still negative
                integral = prev_integral  # don't accumulate



            # #Add limits
            # if  u > max_vel:
            #     u = max_vel

            # elif u < -max_vel:
            #     u = -max_vel
        
            # print("input: ", u)
            if u > 0:
                pwminput = self.maths.map(u, 0, max_vel, 0, 254)
                # if pwminput <15:
                #     pwminput = 15
                self.motor.drive(pwminput,"cw")
            else:
                pwminput = self.maths.map(u, 0, -max_vel, 0, 254)
                # if pwminput < 15:
                #     pwminput = 15
                self.motor.drive(pwminput,"ccw")
            #print(f"pwm input = {pwminput}")
            #update values
            prev_error = error
            prev_integral = integral

            #graph
            self.v.graph(velocity = enVelocity, time = timer )


            #Save data
            data_points.append({"Time":timer, "Velocity":enVelocity})
            
        return data_points
            






#graphing
class Visual:
    t = 0 # Initial Time
    dt = 0.01 # Time steps = 0.01 seconds
    Graph = graph(title = "Velocity", xtitle = "time [s]", ytitle = "velocity RPM", width=800, height=400, fast = False)
    f_velocity = gcurve(color=color.purple, label= "Velocity")
    f_input = gcurve(color=color.blue, label= "input")
    f_error = gcurve(color=color.red, label= "Desired Velocity")


    def graph(self,time, velocity, error=0, input=0):
        
        self.f_velocity.plot(time,velocity)
        print("Time: ",time, "velocity",velocity)
        # self.f_input.plot(time,input)
        # self.f_error.plot(time,error)
        # rate(100)







