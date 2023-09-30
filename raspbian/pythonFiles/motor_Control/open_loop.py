import RPi.GPIO as GPIO
import time


#Class for motor drive
class Motor:
    def __init__(self, enA, in1, in2):
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


#Class for encoder reading
class Encoder:
    def __init__(self, ENCA, ENCB):
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


#velocity calculation class
class VelocityCalculator:
    def __init__(self, CPR):
        self.CPR = CPR
        self.prevTime = 0
        self.prevPos = 0
    
    def calculate(self, currentPos):
        currentTime = time.time()
        timeDiff = currentTime - self.prevTime
        
        if timeDiff > 0:
            velocity = ((currentPos - self.prevPos) / self.CPR) / timeDiff
            self.prevPos = currentPos
            self.prevTime = currentTime
            return velocity
        return 0





#main function
def main():
    GPIO.setmode(GPIO.BCM)
    
    motor = Motor(enA=26, in1=5, in2=6)
    encoder = Encoder(ENCA=7, ENCB=8)
    velocity_calculator = VelocityCalculator(CPR=11)
    
    try:
        while True:
            velocity = velocity_calculator.calculate(encoder.posi)
            
            # Print data
            print(round(velocity * 60))  # RPM
            
            # Drive the motor
            motor.drive(pwmOutput=100) #set motor speed here
            
            time.sleep(1.5)
    
    except KeyboardInterrupt:
        motor.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
