from threading import Thread

import roslibpy
from heiner_comunication.odometry import get_odometry_data_once, OdometryData, reset_odometry
from sensors import alcohol, magnetic, ultrasonic, vibration
from threading import Lock
import time
import battery_voltage as battery_voltage

SENSOR_DATA_CSV_FOLDER = "/home/pi/hard_and_soft/hard-and-soft-2025-frontend/sensor_data/logs/"

CURRENT_THREAD = None
CURRENT_CLIENT = None
CURRENT_MANAGER = None
CURRENT_CSV_FILE:str = None
BATTERY_VOLTAE_INST = None
def start(client: roslibpy.Ros):
    global CURRENT_THREAD, CURRENT_CLIENT, CURRENT_MANAGER, CURRENT_CSV_FILE, BATTERY_VOLTAE_INST
    # Calibrate the sensor
    reset_odometry(client)
    BATTERY_VOLTAE_INST = battery_voltage.SimpleBatteryMonitor(client)
    BATTERY_VOLTAE_INST.connect()

    if CURRENT_THREAD is not None:
        CURRENT_CLIENT = None
        CURRENT_THREAD.join()
    CURRENT_CLIENT = client
    CURRENT_MANAGER = SensorManager()
    CURRENT_THREAD = Thread(target=threadloop, args=(client,))
    CURRENT_CSV_FILE = SENSOR_DATA_CSV_FOLDER + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
    with open(CURRENT_CSV_FILE, "w") as f:
        print("Creating CSV file")
        f.write("Timestamp,Alcohol,MagneticField,Ultrasonic,Vibration,X,Y,Battery Voltage, Battery %\n")
        f.flush()
    CURRENT_THREAD.start()




class SensorData:
    def __init__(self, magnetic_field:float, alcohol:float, ultrasonic:float, vibration:float, odometry:OdometryData, battery_voltage:float, battery_percentage:float):
        self.timestamp: str = time.strftime("%Y-%m-%dT%H:%M:%S")
        self.magnetic_field = magnetic_field
        self.alcohol = alcohol
        self.ultrasonic = ultrasonic
        self.vibration = vibration
        self.x = odometry.position.x
        self.y = odometry.position.y
        self.battery_voltage = battery_voltage
        self.battery_percentage = battery_percentage
    

class SensorManager:
    def __init__(self):
        self.lock = Lock()
        self.data = None

    def update_data(self, data:SensorData):
        print("Updating data")
        print(data)
        with self.lock:
            self.data = data
    
    def get_data(self) -> SensorData:
        with self.lock:
            return self.data
        
    def to_csv_string(self) -> str:
        with self.lock:
            if self.data is None:
                return ""
            return f"{self.data.timestamp},{self.data.alcohol},{self.data.magnetic_field},{self.data.ultrasonic},{self.data.vibration},{self.data.x},{self.data.y},{self.data.battery_voltage},{self.data.battery_percentage}\n"
        
   



def remeasure_data(client: roslibpy.Ros) -> SensorData:
    # Get the sensor data
    time.sleep(0.5)
    magnetic_field = magnetic.magnetic()
    alcohol_v = alcohol.alcohol()
    ultrasonic_v = ultrasonic.ultrasonic()
    vibration_v = vibration.vibration()
    time.sleep(0.5)
    odometry_v = get_odometry_data_once(client)
    time.sleep(0.5)
    BATTERY_VOLTAE_INST.initialize_battery_status()
    voltage = BATTERY_VOLTAE_INST.current_voltage
    percentage = BATTERY_VOLTAE_INST.current_percentage
    time.sleep(0.5)
    CURRENT_MANAGER.update_data(SensorData(magnetic_field, alcohol_v, ultrasonic_v, vibration_v, odometry_v, voltage, percentage))




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
        print("CURRENT_CSV_FILE", CURRENT_CSV_FILE)
        f.write(CURRENT_MANAGER.to_csv_string())
        f.flush()
    

def threadloop(client: roslibpy.Ros):
    global CURRENT_MANAGER
    while CURRENT_CLIENT is not None:
        try:
            time.sleep(0.5)
            remeasure_data(client)
       
            save_current_data_to_csv()
        except Exception as e:
            print(f"Error in threadloop: {e}")
        time.sleep(1)




if __name__ == "__main__":
    client = roslibpy.Ros(host='192.168.149.1', port=9091)

    try:
        client.run()
        if client.is_connected:
            time.sleep(1)
            start(client)

            while True:
                time.sleep(5)
                s = get_current_data()
                if s is None:
                    print("No data available")
                    continue
                print(f"SensorData: {s.battery_percentage}")

        else:
            print("❌ Verbindung zu ROSBridge fehlgeschlagen")
    except KeyboardInterrupt:
        print("⛔️ Beendet durch Tasteneingabe")
        stop_cmd = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist').publish(stop_cmd)
        time.sleep(1)
        client.terminate()
