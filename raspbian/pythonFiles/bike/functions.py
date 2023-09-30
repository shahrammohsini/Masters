#Include the library files
import RPi.GPIO as GPIO
import smbus 
import time
from time import sleep
import json
import copy
import numpy as np
import math
import daqp
import csv
# from vpython import *




class IMU:
    def __init__(self):
        self.maths = Maths()
    # Setup GPIO pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    #some MPU6050 Registers and their Address
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT = 0x3B
    ACCEL_YOUT = 0x3D
    ACCEL_ZOUT = 0x3F
    GYRO_XOUT  = 0x43
    GYRO_YOUT  = 0x45
    GYRO_ZOUT  = 0x47


    bus = smbus.SMBus(1) # or bus = smbus.SMBus(0) for older version boards
    Device_Address = 0x68 # MPU6050 device address




    def MPU_Init(self):
        
         # write to sample rate register
        self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)

        # Write to power management register
        self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)

        # Write to Configuration register
        self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)

        # Write to Gyro configuration register
        self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)

        # Write to interrupt enable register
        self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def read_raw_data(self,addr):
        #Accelero and Gyro value are 16-bit
            high = self.bus.read_byte_data(self.Device_Address, addr)
            low = self.bus.read_byte_data(self.Device_Address, addr+1)
        
            #concatenate higher and lower value
            value = ((high << 8) | low)
            
            #to get signed value from mpu6050
            if(value > 32768):
                    value = value - 65536
            return value
        

    def data(self):
          self.MPU_Init()
          
          while True:
            #Read Accelerometer raw value
            acc_x = self.read_raw_data(self.ACCEL_XOUT)
            acc_y = self.read_raw_data(self.ACCEL_YOUT)
            acc_z = self.read_raw_data(self.ACCEL_ZOUT)

            #Read Gyroscope raw value
            gyro_x = self.read_raw_data(self.GYRO_XOUT)
            gyro_y = self.read_raw_data(self.GYRO_YOUT)
            gyro_z = self.read_raw_data(self.GYRO_ZOUT)

            Ax = acc_x/16384.0
            Ay = acc_y/16384.0 #really only need this one
            Az = acc_z/16384.0

            Gx = gyro_x/131.0
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0

        # Uncomment below line to see the Accelerometer and Gyroscope values   
            # print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
            in_min = 1
            in_max = -1
            out_min = 0
            out_max = 180
            
            # setAngle() # Use this function to set the servo motor point
            
            # Convert accelerometer Y axis values from 0 to 180   
            value = (Ay - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
            value = int(value)
            # print(value)
            if value >= 0 and value <= 180:
                sleep(0.08)
            
            if value >= 90:
                value = self.maths.map(value, 90, 180, 0, 90)
            else:
                value = self.maths.map(value, 90, 0, 0, -90)

            return value



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
    def init(self):
        self.pwm_pin = 26
        self.brake_pin = 5
        self.direction_pin = 6
        self.setup()
        
    def setup(self):
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.brake_pin, GPIO.OUT)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 1000)  #send speed signal to motor (from 1 to 100)
        self.pwm.start(100) #Start initial speed at 0. Numbers are flipped for some reason so 100 means 0
        
    def drive(self, pwmOutput, direction):
        GPIO.output(self.brake_pin, GPIO.HIGH)  #Disable Brake
        if direction == "ccw":
            GPIO.output(self.direction_pin, GPIO.HIGH)
            
        else:
            GPIO.output(self.direction_pin, GPIO.LOW)
           
        #Note, on this motor the values of pwm are flipped. 255 means stop, 0 means go full speed so I subtracted from 100 to flip it
        self.pwm.ChangeDutyCycle(100 - (pwmOutput / 255.0 * 100)) #change pwm value from 1 to 100 and send it to the motor 
        # self.pwm.ChangeDutyCycle(50) #change pwm value from 1 to 100 and send it to the motor
        
    def stop(self):
        self.pwm.stop()







"""
Purpose: Create the Dynamic Matrix
Variables: voltage, PWM_val, loop_time
How to use: Instantiate the DynamicMatrix class as argument and then call the any of its functions to use it
Example: dynamic_matrix = f.DynamicMatrix()  
         dynamic_matrix.data_writer(voltage=6, pwm_val = 150, loop_time = 20) //collect open loop data for 6v, 150 pwm, and 20 sec into a json file
"""
class DynamicMatrix:

    def write_to_array(self,pwm_val, stop_angle):  #Drive the motor and collect data into an array
        GPIO.setmode(GPIO.BCM)
        data_points = []
        startTime = time.time()
        imu = IMU()
        motor = Motor()
        try:
            print(imu.data())
            while imu.data() > stop_angle:
                angle = imu.data()
                time_t = time.time() - startTime
                # Print data
                print(f"Time:{round(time_t,2)}", f"Angle:{angle}")
                #Enter Data into array
                data_points.append({"Time":round(time_t,2), "Angle":round(angle,2)})
                # Drive the motor
                motor.drive(pwmOutput = pwm_val, direction = "ccw") #set motor speed here
                
                # time.sleep(0.1)
            #stop motor after the loop    
            motor.stop()
            GPIO.cleanup()
            print("exited loop")
        except KeyboardInterrupt:
            motor.stop()
            GPIO.cleanup()

        return data_points

    #Enter Data into json file
    def data_writer(self,voltage, pwm_val, stop_angle):
        data_points = []
        data_points = self.write_to_array(pwm_val, stop_angle)
        print("entered back into data_writer")
        try:
        
            with open(f"/home/sham/Desktop/pythonFiles/bike/Data/{voltage}_volts.json", mode="w") as file:
                json.dump(data_points, file, indent=1)
                print("dumping data")
            file.close
        except Exception as e:
            print(f"Error while writing data to the file: {e}")


    
    #Create the A matrix
    def A_Matrix(self, voltage, nu):
        with open(f"/home/sham/Desktop/pythonFiles/bike/Data/{voltage}_volts.json", mode='r') as file:
            data_points = json.load(file)
        angle_Array = [data_point["Angle"] for data_point in data_points]
        # Divide every value in angle_Array by voltage to normalize
        angle_Array = [round(v / voltage,2) for v in angle_Array]

        # velocity_Array = [1,2,3,4,5,6,7,8,9]
        nu = 3
        r = len(angle_Array) #get number of rows
        Mod_Array = copy.deepcopy(angle_Array)
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



    #Create the A matrix from csv file
    def A_Matrix_csv(self, nu):
        
        angle_Array=[]
        with open("/home/sham/Desktop/pythonFiles/bike/Data/Open_Loop_Data_8.csv", "r") as csv_file:
            csv_reader = csv.reader(csv_file)

            for line in csv_reader:
                angle_Array.append(abs(round(float(line[0]),4)))

        # Divide every value in angle_Array by voltage to normalize
        # angle_Array = [round(v / voltage,2) for v in angle_Array]

        # velocity_Array = [1,2,3,4,5,6,7,8,9]
        nu = 3
        r = len(angle_Array) #get number of rows
        Mod_Array = copy.deepcopy(angle_Array)
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
        self.DM = DynamicMatrix()
        self.maths = Maths()
        # self.v = Visual()
        self.imu = IMU()
        self.motor = Motor()

    def controller(self, voltage, nu):
        #Constants
        # A = self.DM.A_Matrix(voltage , nu) #get the A matrix
        A = self.DM.A_Matrix_csv(nu) #get the A matrix
        N = len(A)
        nu = 3 #control horizon
        u = np.zeros(N) #Voltage (Step input)for model calculation
        Y = 0
    
        PHI = np.zeros(nu) #To correct model inaccuracy
        LAMBDA = 1.3 #penalty factor
        # y_mes = 0  #Measured position
        # error = np.zeros((nu, 1)) # r_desired_vel - y_hat  #error between desired velocity and predicted velocity
        error = 0
        r_desired_vel = 0 #Degrees  Desired angles
        delta_u = np.zeros(N) #Optimized change in input (u) to minimize error 
        u_prev = np.zeros(N) # input from previous iteration
        I_Matrix = np.identity(nu) # create an identity matrix of size 3 by 3
        A_T = np.transpose(A)  #Transpose of matrix A
        y_hat = delta_u.dot(A) #Predicted position
        
        startTime = time.time()

        while True:
            time.sleep(0.2)
            timer = time.time() - startTime #Timer for graph
            angle = self.imu.data() # get bike angle from acceleromter
            print("angle: ",angle)
            ym = angle


            #MPC
            PHI = ym - y_hat #difference between calculated velocity (y_hat) and measured velocity (ym) to see if the model is inaccurate
            
            # print("y_hat_a: ",y_hat)
            y_hat = y_hat + PHI #correct model inacuracy

            error = r_desired_vel - y_hat
            # print("error: ",error)
            delta_u = error.dot(np.linalg.inv(A_T.dot(A) - (LAMBDA * I_Matrix)).dot(A_T)) #optimize input required to remove error
            u = u_prev + delta_u # update input with optimized change
            # Add limit control on input U
            print("input", u[0])
            if u[0] > 90:
                u[0] = 90

            elif u[0] < -90:
                u[0] = -90
        
            delta_u = u - u_prev #This recalculates delta_u so that it includes the limits added for the next step where we calculate y_hat
            #print(delta_u.size)
            
            y_hat = y_hat + delta_u.dot(A) #update predicted output (note the delta_u used includes the limits added)
            y_hat[:-1] = y_hat[1:] #This line only moves the values of y_hat one to the right

            if u[0] > 0:
                pwminput = self.maths.map(u[0], 0, 90, 0, 255)
                self.motor.drive(pwminput,"ccw")
            else:
                pwminput = self.maths.map(u[0], 0, -90, 0, 255)
                self.motor.drive(pwminput,"cw")
            print(f"pwm input = {pwminput}")
            #update values
            u_prev = u

            #graph
            # self.v.graph(velocity = enVelocity, time = timer, error = r_desired_vel, input = u[1] )




class Optimizer_Quadratic_Problem:
    def __init__(self, A, B, C, Q, R, N):
        self.A = A
        self.B = B
        self.C = C
        self.Q = Q
        self.R = R
        self.N = N
        
        self.nx = A.shape[0]
        self.nu = B.shape[1]

    def form_mpc_problem(self, x0):
    # Formulate the QP problem matrices using the state-space model, Q, R, constraints, etc.
        
        # Constructing H matrix
        H_block_state = np.kron(np.eye(self.N), self.Q)
        H_block_input = np.kron(np.eye(self.N), self.R)
        H = np.block([[H_block_state, np.zeros((self.N*self.nx, self.N*self.nu))], [np.zeros((self.N*self.nu, self.N*self.nx)), H_block_input]])

        # Constructing f vector
        f = np.zeros(self.N * (self.nx + self.nu))

        # Constructing system dynamics over the horizon
        A_dynamics = np.zeros((self.N*self.nx, self.N*self.nu))
        for i in range(self.N):
            A_dynamics[i*self.nx:(i+1)*self.nx, :i*self.nu] = np.linalg.matrix_power(self.A, i+1) @ self.B
            
        # A matrix for input constraints
        A_input = np.kron(np.eye(self.N), np.eye(self.nu))
        A = np.vstack((A_dynamics, A_input))
        
        # b_upper and b_lower vectors for the input constraints
        b_upper = np.concatenate((np.tile(self.A @ x0, self.N), np.tile(180, self.N*self.nu)))
        b_lower = np.concatenate((np.tile(self.A @ x0, self.N), np.tile(-180, self.N*self.nu)))
        
        # sense array (assuming 1 represents <= in the daqp solver)
        sense = np.tile(1, A.shape[0])  # Match the number of rows in A

        return H, f, A, b_upper, b_lower, sense



    def compute_control(self, x0):
        H, f, A, b_upper, b_lower, sense = self.form_mpc_problem(x0)
        

        print("compute_control", sense)
        x_star, fval, exitflag, info = daqp.solve(H, f, A, b_upper, b_lower, sense)

        # Assuming the control sequence is u_0, u_1, ..., u_{N-1}, we return the first control action
        u0 = x_star[:self.nu]

        # print("u", u0)

        return u0







class MPC_controller_StateSpace:
    def __init__(self):
        self.DM = DynamicMatrix()
        self.maths = Maths()
        self.imu = IMU()
        self.motor = Motor()


        # constatnts:
        g = 9.8
        rR = 0.06 #Radius of Reaction wheel
        L = 0.133 # Length of Rod 0.17
        mR = 0.027 # mass of rod in kg          
        mF = 0.060 # mass of reaction wheel in kg    
        mm = 0.11
        M = 0.5*mR + mF
        IR = (1/3)*mR*L*L  #Moment of interti of rod
        Ip = (1/3)*mR*rR*rR
        I = Ip + (mR*L*L)
        #Motor
        kt = 0.05 #Nm/A Torque constant of motor. This is an educated guess and the value can be anywhere between 0.05 to 0.2
        k = kt #Also a guess
        R = 0.1 #internal resistance of the motor. Guessed it. It can be anywhere from 0.1 to 1 ohm
        inductance = 0.00001 #internal inductance. Anywhere from 0.00001 to 0.00005 henrys
        q1 = 90
        q2 = 1
        q3 = 1
        q4 = 1
        r = 1

        # State-space matrices and weights
        self.A = np.array([[0,1,0,0],[(M*g/L), 0 , 0, (-kt/I)], [0, 0, 0, (kt/IR)], [0,(k/inductance),(-k/inductance),(-R/inductance)]])
        self.B = np.array([[0], [0], [0], [1/L]])
        self.C = np.array([[1, 0, 0, 0],[0, 1 , 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        self.Q = np.diag([q1, q2, q3, q4])
        self.R = np.array([[r]])
        self.N = 10  # Prediction horizon

        # Initialize the optimizer
        self.optimizer = Optimizer_Quadratic_Problem(self.A, self.B, self.C, self.Q, self.R, self.N)

    def controller(self):
        desired_angle = 90
        while True:
            # Measure the current angle from the IMU
            measured_angle = self.imu.data()
            
            # Calculate the error between desired and measured angles
            error = desired_angle - measured_angle
            print("Error: ", error)

            # The state vector x0 will now start with the error
            x0 = np.array([error, 0, 0, 0])  
            print("State x0: ", x0)

            # Get the optimal control action using MPC
            u0 = self.optimizer.compute_control(x0)
            print("Optimal control action u0: ", u0)

            pwm_val = self.maths.map(u0, 0, 180, 0, 255) # Convert control action to PWM (assuming u0 is in the range of [-180,180])
            if pwm_val > 0:
                self.motor.drive(pwmOutput=pwm_val, direction="cw")
            else:
                self.motor.drive(pwmOutput=-pwm_val, direction="ccw")

            time.sleep(0.1)  # You might need to adjust the control rate as required.
