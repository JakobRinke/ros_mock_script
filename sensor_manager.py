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
CURRENT_CSV_FILE = None
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
        self.x = odometry.x
        self.y = odometry.y
        self.battery_voltage = battery_voltage
        self.battery_percentage = battery_percentage
    

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
        
   



def remeasure_data(client: roslibpy.Ros) -> SensorData:
    # Get the sensor data
    magnetic_field = magnetic.magnetic()
    alcohol_v = alcohol.alcohol()
    ultrasonic_v = ultrasonic.ultrasonic()
    vibration_v = vibration.vibration()
    odometry_v = get_odometry_data_once(client)
    BATTERY_VOLTAE_INST.initialize_battery_status()
    voltage = BATTERY_VOLTAE_INST.current_voltage
    percentage = BATTERY_VOLTAE_INST.current_percentage
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
        f.write(CURRENT_MANAGER.to_csv_string())

def threadloop(client: roslibpy.Ros):
    global CURRENT_MANAGER
    while CURRENT_CLIENT is not None:
        remeasure_data(client)
        CURRENT_MANAGER.get_data()
        # Sleep for a short duration to avoid busy waiting
        save_current_data_to_csv()
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
