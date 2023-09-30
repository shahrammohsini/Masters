from gpiozero import LED
import time 

led = LED(18); #set pin 18 to led

while True:
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)