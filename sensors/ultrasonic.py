import RPi.GPIO as GPIO
import time

def ultrasonic_init():
    GPIO.setmode(GPIO.BOARD)

    TRIG = 18
    ECHO = 16
    
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(2)

    
MAX_DISTANCE = 400  # Maximum distance in cm for the ultrasonic sensor
def ultrasonic():
    TRIG = 18
    ECHO = 16
    distance = 0
    try:


        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)

        timeout = time.time() + 1  # 1 second timeout
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if time.time() > timeout:
                return MAX_DISTANCE

        timeout = time.time() + 1  # 1 second timeout
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
            if time.time() > timeout:
                return MAX_DISTANCE

        pulse_duration = pulse_end-pulse_start

        distance = pulse_duration * 17150

        distance = round(distance,2)

    except Exception as e:
        print(f"Error in ultrasonic sensor: {e}")
    
    GPIO.cleanup()

    return distance
