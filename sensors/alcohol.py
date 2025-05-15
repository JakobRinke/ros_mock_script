import RPi.GPIO as GPIO
import time

def alcohol():
    DIGITAL_INPUT=22
    
    GPIO.setup(DIGITAL_INPUT,GPIO.IN)

    try:
        result=1-GPIO.input(DIGITAL_INPUT)

    finally:
        pass
        
    return result
