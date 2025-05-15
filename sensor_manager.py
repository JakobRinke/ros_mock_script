from threading import Thread
import json
import roslibpy
from heiner_comunication.odometry import get_odometry_data_once, OdometryData, reset_odometry
from sensors import alcohol, magnetic, ultrasonic, vibration
from threading import Lock
import time
import battery_voltage as battery_voltage
import traceback
import RPi.GPIO as GPIO
import heiner_comunication.topic_getter as topic_getter


GPIO.setmode(GPIO.BOARD)

SENSOR_DATA_CSV_FOLDER = "/home/pi/hard_and_soft/hard-and-soft-2025-frontend/sensor_data/logs/"

CURRENT_THREAD:Thread = None
CURRENT_CLIENT = None
CURRENT_MANAGER = None
CURRENT_CSV_FILE:str = None
BATTERY_VOLTAE_INST = None
WRITE_FILE_THREAD:Thread = None
WRITE_ROS_THREAD:Thread = None

def start(client: roslibpy.Ros):
    global CURRENT_THREAD, CURRENT_CLIENT, CURRENT_MANAGER, CURRENT_CSV_FILE, BATTERY_VOLTAE_INST, WRITE_FILE_THREAD, WRITE_ROS_THREAD
    # Calibrate the sensor
    ultrasonic.ultrasonic_cal()
    reset_odometry(client)
    BATTERY_VOLTAE_INST = battery_voltage.SimpleBatteryMonitor(client)
    BATTERY_VOLTAE_INST.connect()

    if CURRENT_THREAD is not None:
        CURRENT_CLIENT = None
        CURRENT_THREAD.join()
    if WRITE_FILE_THREAD is not None:
        WRITE_FILE_THREAD.join()
    if WRITE_ROS_THREAD is not None:
        WRITE_ROS_THREAD.join()

    CURRENT_CLIENT = client
    CURRENT_MANAGER = SensorManager()
    CURRENT_THREAD = Thread(target=sensorloop, args=(client,))
    WRITE_FILE_THREAD = Thread(target=writeloop)
    WRITE_ROS_THREAD = Thread(target=write_into_ros, args=(client,))
    CURRENT_CSV_FILE = SENSOR_DATA_CSV_FOLDER + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
    with open(CURRENT_CSV_FILE, "w") as f:
        f.write("Timestamp,Alcohol,MagneticField,Ultrasonic,Vibration,X,Y,Battery Voltage, Battery %\n")
        f.flush()
    CURRENT_THREAD.start()
    WRITE_FILE_THREAD.start()
    WRITE_ROS_THREAD.start()




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

    def __str__(self):
        return (
            f"SensorData(\n"
            f"  timestamp={self.timestamp},\n"
            f"  alcohol={self.alcohol},\n"
            f"  magnetic_field={self.magnetic_field},\n"
            f"  ultrasonic={self.ultrasonic},\n"
            f"  vibration={self.vibration},\n"
            f"  x={self.x},\n"
            f"  y={self.y},\n"
            f"  battery_voltage={self.battery_voltage},\n"
            f"  battery_percentage={self.battery_percentage}\n"
            f")"
        )

    def to_dict(self):
        return {
            "timestamp": self.timestamp,
            "alcohol": self.alcohol,
            "magnetic_field": self.magnetic_field,
            "ultrasonic": self.ultrasonic,
            "vibration": self.vibration,
            "x": self.x,
            "y": self.y,
            "battery_voltage": self.battery_voltage,
            "battery_percentage": self.battery_percentage
        }    
    


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
            return f"{self.data.timestamp},{self.data.alcohol},{self.data.magnetic_field},{self.data.ultrasonic},{self.data.vibration},{self.data.x},{self.data.y},{self.data.battery_voltage},{self.data.battery_percentage}\n"
    


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
        print(CURRENT_MANAGER.to_csv_string())
        f.write(CURRENT_MANAGER.to_csv_string())
        f.flush()
    
def get_senor_data_from_ros_once(client: roslibpy.Ros) -> SensorData:
    t = topic_getter.get_topic_once(
        client=client,
        topic_name='/sensor_data',
        message_type='std_msgs/String',
        timeout=5
    )
    if t is None:
        raise Exception("No data available")
    data = json.loads(t)
    return SensorData(
        magnetic_field=data['magnetic_field'],
        alcohol=data['alcohol'],
        ultrasonic=data['ultrasonic'],
        vibration=data['vibration'],
        odometry=OdometryData(
            position=OdometryData.Position(
                x=data['x'],
                y=data['y']
            )
        ),
        battery_voltage=data['battery_voltage'],
        battery_percentage=data['battery_percentage']
    )

def sensorloop(client: roslibpy.Ros):
    global CURRENT_MANAGER
    while CURRENT_CLIENT is not None:
        try:
            magnetic_field = magnetic.magnetic()
            alcohol_v = alcohol.alcohol()
            ultrasonic_v = ultrasonic.ultrasonic()
            vibration_v = vibration.vibration()

            odometry_v = get_odometry_data_once(client)

            BATTERY_VOLTAE_INST.initialize_battery_status()
            voltage = BATTERY_VOLTAE_INST.current_voltage
            percentage = BATTERY_VOLTAE_INST.current_percentage

            CURRENT_MANAGER.update_data(SensorData(magnetic_field, alcohol_v, ultrasonic_v, vibration_v, odometry_v, voltage, percentage))
        except Exception as e:
            print(f"Error in threadloop: {e}")
        time.sleep(0.1)


def writeloop():
    global CURRENT_MANAGER
    while CURRENT_CLIENT is not None:
        try:
            time.sleep(1)
            save_current_data_to_csv()
        except Exception as e:
            print(f"Error in writeloop: {e}")
        time.sleep(1)

def write_into_ros(client: roslibpy.Ros):
    global CURRENT_MANAGER
    top = roslibpy.Topic(client, '/sensor_data', 'std_msgs/String')
    top.advertise()
    while CURRENT_CLIENT is not None:
        try:
            time.sleep(1)
            data = get_current_data()
            if data is None:
                print("No data available")
                continue
            data_dict = data.to_dict()
            data_string = json.dumps(data_dict)
            top.publish(roslibpy.Message({"data": data_string}))
            

        except Exception as e:
            print(f"Error in write_into_ros: {e}")
            traceback.print_exc()
        time.sleep(0.1)
    try:
        top.unadvertise()
    except Exception as e:
        pass




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
                #print(f"SensorData: {s.battery_percentage}")

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
