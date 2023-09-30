import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)  # Suppress GPIO warnings
GPIO.cleanup()           # Clean any previous configuration
# Pin Definitions
enA = 26
in1 = 5
in2 = 6

pwm_value = 45

def setup():
    GPIO.setmode(GPIO.BCM)  # Set the mode to BCM numbering

    # Set up pins
    GPIO.setup(enA, GPIO.OUT)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)

    # Set initial rotation direction
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    return GPIO.PWM(enA, 1000)  # Set PWM frequency to 1000 Hz  //how often the signal is updated

def loop(pwm):
    # Drive the motor in one direction
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    # Set the desired speed (PWM value)
    pwmOutput = pwm_value / 255.0  # Convert to duty cycle (0.0 - 1.0)
    pwm.ChangeDutyCycle(pwmOutput * 100)  # Convert to percentage

try:
    motor_pwm = setup()
    motor_pwm.start(0)  # Start PWM with 0% duty cycle
    while True:
        loop(motor_pwm)
        time.sleep(0.1)  # Just to prevent unnecessary CPU usage
except KeyboardInterrupt:
    motor_pwm.stop()  # Stop the PWM
    GPIO.cleanup()  # Cleanup GPIO settings
