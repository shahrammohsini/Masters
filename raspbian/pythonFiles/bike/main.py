import functions as f
import RPi.GPIO as GPIO
import time
import numpy as np

#main function
def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.cleanup()

    imu = f.IMU()
    motor = f.Motor()
    dynamic_matrix = f.DynamicMatrix()
    mpc = f.MPC_controller()
    mpc_ss = f.MPC_controller_StateSpace()

    
    
    try:

        print("Code started")

        # time.sleep(1)
        # dynamic_matrix.data_writer(voltage = 100, pwm_val = 100, stop_angle= 90)
        # print(dynamic_matrix.A_Matrix(voltage=11.1, nu = 3))
        mpc.controller(voltage=100, nu = 3)
        # time.sleep(1)
        # while True:
        #     print(imu.data())
        #     motor.drive(pwmOutput= 255, direction= "ccw")
        #     time.sleep(0.1)

        # mpc_ss.controller()


        # g = 9.8
        # rR = 0.06 #Radius of Reaction wheel
        # L = 0.133 # Length of Rod 0.17
        # mR = 0.027 # mass of rod in kg          
        # mF = 0.060 # mass of reaction wheel in kg    
        # mm = 0.11
        # M = 0.5*mR + mF
        # IR = (1/3)*mR*L*L  #Moment of interti of rod
        # Ip = (1/3)*mR*rR*rR
        # I = Ip + (mR*L*L)
        # kt = 0.05 #Nm/A Torque constant of motor. This is an educated guess and the value can be anywhere between 0.05 to 0.2
        # k = kt #Also a guess

        # # Define your state-space model, weight matrices, and constraints here
        # A = np.array([[0,1,0,0],[(M*g/L), 0 , 0, (-kt/I)], [0, 0, 0, (kt/IR)], [0,(k/L),(-k/L),(-R/L)]])
        # B = np.array([[0], [0], [0], [1/L]])
        # C = np.array([[1, 0, 0, 0],[0, 1 , 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        # q1 = 1
        # q2 = 1
        # q3 = 1
        # q4 = 1
        # r = 1
        # #When Q is diagonal matrix it means you're ignoring penalization for interactions between states. (Same is true for R if you have multiple inputs)
        # Q = np.diag([q1, q2, q3, q4]) #Penalizes changes in state. Higher q means higher penalization for that state
        # R = np.array([[r]]) #Penalizes changes in input. Higher r means higher penalization for that input
        # N = 10  # Prediction horizon

        # optimizer = f.Optimizer_Quadratic_Problem(A, B, C, Q, R, N)
        # x0 = np.array([0, 0, 0, 0])  # Current state. Assuming everythign starts at 0 you put all zeros. (Might have to change the first one to 90?)
        # u0 = optimizer.compute_control(x0)
        # print("Optimal control action:", u0)



        
    
    except KeyboardInterrupt:
        print("Code ended")
        motor.stop()
        GPIO.cleanup()
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()

        