import RPi.GPIO as GPIO
import time

def vibration():
    DIGITAL_PIN=33


    # Set Switch GPIO as input
    GPIO.setup(DIGITAL_PIN,GPIO.IN)

    try:
        result=GPIO.input(DIGITAL_PIN)

    finally:
        # Reset GPIO settings
        pass

    return result
