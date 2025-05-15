# Import required libraries
import RPi.GPIO as GPIO
import time

def magnetic():
    DIGITAL_PIN=37

    # Tell GPIO library to use GPIO references


    # Set Switch GPIO as input
    GPIO.setup(DIGITAL_PIN,GPIO.IN,pull_up_down=GPIO.PUD_OFF)

    try:
        result=GPIO.input(DIGITAL_PIN)
    
    finally:
        pass

    return result
