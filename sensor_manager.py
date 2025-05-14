from threading import Thread

import roslibpy
from heiner_comunication.odometry import get_odometry, OdometryData, reset_odometry
from sensors import alcohol, magnetic, ultrasonic, vibration
from threading import Lock
import time

SENSOR_DATA_CSV_FOLDER = "/home/pi/hard_and_soft/hard-and-soft2025-frontend/sensor_data/log/"

CURRENT_THREAD = None
CURRENT_CLIENT = None
CURRENT_MANAGER = None
CURRENT_CSV_FILE = None

def start(client: roslibpy.Ros):
    global CURRENT_THREAD, CURRENT_CLIENT, CURRENT_MANAGER, CURRENT_CSV_FILE
    # Calibrate the sensor
    reset_odometry(client)
    ultrasonic.ultrasonic_cal()
    if CURRENT_THREAD is not None:
        CURRENT_CLIENT = None
        CURRENT_THREAD.join()
    CURRENT_CLIENT = client
    CURRENT_MANAGER = SensorManager()
    CURRENT_THREAD = Thread(target=threadloop, args=(client,))
    CURRENT_CSV_FILE = SENSOR_DATA_CSV_FOLDER + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
    with open(CURRENT_CSV_FILE, "w") as f:
        f.write("Timestamp,Alcohol,MagneticField,Ultrasonic,Vibration,X,Y\n")
        f.flush()
    CURRENT_THREAD.start()




class SensorData:
    def __init__(self, magnetic_field:float, alcohol:float, ultrasonic:float, vibration:float, odometry:OdometryData):
        self.timestamp: str = time.strftime("%Y-%m-%dT%H:%M:%S")
        self.magnetic_field = magnetic_field
        self.alcohol = alcohol
        self.ultrasonic = ultrasonic
        self.vibration = vibration
        self.x = odometry.x
        self.y = odometry.y


class SensorManager:
    def __init__(self):
        self.lock = Lock()
        self.data = None

    def update_data(self, data:SensorData):
        with self.lock:
            self.data = data
    
    def get_data(self) -> SensorData:
        with self.lock:
            return self.data
        
    def to_csv_string(self) -> str:
        with self.lock:
            if self.data is None:
                return ""
            return f"{self.data.timestamp},{self.data.alcohol},{self.data.magnetic_field},{self.data.ultrasonic},{self.data.vibration},{self.data.x},{self.data.y}\n"



def remeasure_data(client: roslibpy.Ros) -> SensorData:
    # Get the sensor data
    magnetic_field = magnetic.magnetic(client)
    alcohol_v = alcohol.alcohol(client)
    ultrasonic_v = ultrasonic.ultrasonic(client)
    vibration_v = vibration.get_vibration_data_once(client)
    odometry_v = get_odometry(client)

    CURRENT_MANAGER.update_data(SensorData(magnetic_field, alcohol_v, ultrasonic_v, vibration_v, odometry_v))

def get_current_data() -> SensorData:
    global CURRENT_MANAGER
    if CURRENT_MANAGER is None:
        raise Exception("SensorManager not started")
    return CURRENT_MANAGER.get_data()

def save_current_data_to_csv():
    global CURRENT_MANAGER, CURRENT_CSV_FILE
    if CURRENT_MANAGER is None:
        raise Exception("SensorManager not started")
    with open(CURRENT_CSV_FILE, "a") as f:
        f.write(CURRENT_MANAGER.to_csv_string())

def threadloop(client: roslibpy.Ros):
    global CURRENT_MANAGER
    while CURRENT_CLIENT is not None:
        remeasure_data(client)
        CURRENT_MANAGER.get_data()
        # Sleep for a short duration to avoid busy waiting
        save_current_data_to_csv()
        time.sleep(1)




