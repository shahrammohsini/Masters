import RPi.GPIO as GPIO
import functions as f
import time




#main function
def main():
    GPIO.setmode(GPIO.BCM)
    
    # motor = f.Motor(enA=26, in1=5, in2=6)
    # encoder = f.Encoder(ENCA=7, ENCB=8)
    # velocity_calculator = f.VelocityCalculator(CPR=11)
    dynamic_matrix = f.DynamicMatrix()
    try:
        
        # dynamic_matrix.data_writer(voltage=6, pwm_val = 150, loop_time = 20)
        A = dynamic_matrix.A_Matrix(nu = 3, voltage = 6)
        print("hello")
        print(A)
    
    except KeyboardInterrupt:
        # motor.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
