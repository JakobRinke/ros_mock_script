import roslibpy
import math
import time
from heiner_comunication.lidar import get_lidar_data_once
from heiner_comunication.motor_control import move, rotate
import math
import time

# Parameter
FORWARD_SPEED = 1.0  # Geschwindigkeit beim Vorwärtsfahren
ROTATE_SPEED = 1.0   # Geschwindigkeit beim Drehen
FORWARD_DURATION = 0.5  # Dauer für kleine Vorwärtsbewegung
TURN_DURATION = 0.5     # Dauer für kleine Drehung

# Schwellwerte
FRONT_ANGLE = 0
FRONT_SPREAD = math.radians(30)  # +-30 Grad
RIGHT_ANGLE = -math.pi / 2
RIGHT_SPREAD = math.radians(30)  # +-30 Grad
DISTANCE_THRESHOLD = 0.24  # Mindestabstand in Meter
RIGHT_OPEN_THRESHOLD = 1.5  # Mehr Abstand für rechts nötig


def get_distance(lidar, angle, spread):
    """Gibt den durchschnittlichen Abstand um einen bestimmten Winkel."""
    return lidar.get_value_around_angle_min(angle, spread)


def is_front_clear(lidar):
    distance = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
    return distance > DISTANCE_THRESHOLD


def is_right_clear(lidar):
    distance = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
    return distance > RIGHT_OPEN_THRESHOLD


def move_forward(client):
    move(client, FORWARD_SPEED, 0, FORWARD_DURATION)


def rotate_right(client):
    rotate(client=client, speed=-ROTATE_SPEED, timeout=TURN_DURATION)


def rotate_left(client):
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION)


def wall_follower(client):
    """Verbesserter Right-Hand Wall Follower Algorithmus."""
    while True:
        lidar = get_lidar_data_once(client, True)
        
        if is_right_clear(lidar):
            rotate_right(client)
            move_forward(client)
        elif is_front_clear(lidar):
            move_forward(client)
        else:
            rotate_left(client)


def main(client):
    wall_follower(client)


# --- Einstiegspunkt ---
if __name__ == "__main__":
    client = roslibpy.Ros(host='192.168.149.1', port=9091)
    try:
        client.run()
        if client.is_connected:
            main(client)
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
