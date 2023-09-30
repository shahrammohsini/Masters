import RPi.GPIO as GPIO
import functions as f
import time
import json
# import vpython




#main function
def main():
    GPIO.setmode(GPIO.BCM)
    
    motor = f.Motor(enA=26, in1=5, in2=6)
    encoder = f.Encoder(ENCA=7, ENCB=8)
    velocity_calculator = f.VelocityCalculator()
    dynamic_matrix = f.DynamicMatrix()
    mpc = f.MPC_controller()
    pid = f.PID_controller()
    # v = f.Visual()
    try:
        # print(dynamic_matrix.A_Matrix(voltage= 10, nu = 4))
        data_points = dynamic_matrix.data_writer(voltage=12, pwm_val = 255, loop_time =8 ,time_step = 0.078)
        # A = dynamic_matrix.A_Matrix(nu = 3, voltage = 6)
        print("hello")
        # data_points = []
        # # print(A)
        # data_points = mpc.controller(voltage = 12, nu = 6 , desired_vel= 2000, loop_time = 15, normalize_val=255, time_Step = 0.18, LAMBDA =1.1)
        

        # data_points = pid.controller(desired_vel= 2000, loop_time = 15, kp = 1.5, ki = 0.9, kd = 0.05)

        print(data_points)
        motor.stop()
        file_name = input("Enter file name: ")
        try:
        
            with open(f"motor_Control/Data/Run_Data/MPC_Data/{file_name}.json", mode="w") as file:
                json.dump(data_points, file, indent=1)
            file.close
        except Exception as e:
            print(f"Error while writing data to the file: {e}")






        # while True:
        #     # rate(100)
        #     v.graph(velocity = 1, time = 1 , error = 2, input = 3)

        # while True:
        #     motor.drive(40,"cw")
        # while True:
        #     motor.drive(255,"cw")
        #     print(velocity_calculator.calculate(encoder.posi))
        # motor.stop()

        # while True:
        #     motor.drive(110,"cw")
        #     # print("m2: " ,encoder.velocity)
        #     print("m1: ",velocity_calculator.calculate(encoder.posi))
        #     # time.sleep(1)
        # while True:
        #     motor.drive(50,"cw")
        #     print((encoder.posi)/11)

        motor.stop()

        # velocity_calculator.calculate(encoder.)
        # mpc.motor.drive(255)
    
    except KeyboardInterrupt:
        motor.stop()
        print("motor")
        GPIO.cleanup()

if __name__ == "__main__":
    main()
