import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
def alcohol():
    DIGITAL_INPUT=22
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(DIGITAL_INPUT,GPIO.IN)

    try:
        result=1-GPIO.input(DIGITAL_INPUT)

    finally:
        GPIO.cleanup()
        
    return result
