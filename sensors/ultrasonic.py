import RPi.GPIO as GPIO
import time

def ultrasonic_cal():
    TRIG = 18
    ECHO = 16
    
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(2)

def ultrasonic():
    TRIG = 18
    ECHO = 16
    
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    
    GPIO.output(TRIG, GPIO.LOW)

    try:
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)

        timeout = 0.1
        start_time = time.time()
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if pulse_start - start_time > timeout:
                return 400

        start_time = time.time()
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
            if pulse_end - start_time > timeout:
                return 400

        pulse_duration = pulse_end-pulse_start

        distance = pulse_duration * 17150

        distance = round(distance,2)

    finally:
        pass

    return distance