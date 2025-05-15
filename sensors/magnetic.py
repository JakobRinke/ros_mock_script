# Import required libraries
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
def magnetic():
    DIGITAL_PIN=37

    # Tell GPIO library to use GPIO references
    GPIO.setmode(GPIO.BOARD)

    # Set Switch GPIO as input
    GPIO.setup(DIGITAL_PIN,GPIO.IN,pull_up_down=GPIO.PUD_OFF)

    try:
        result=GPIO.input(DIGITAL_PIN)
    
    finally:
        GPIO.cleanup()

    return result
