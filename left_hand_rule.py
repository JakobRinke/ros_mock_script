import roslibpy
import math
import time
from heiner_comunication.lidar import get_lidar_data_once
from heiner_comunication.motor_control import move, rotate

# --- Konfiguration ---
BASE_SPEED = 0.2
ROTATE_SPEED = 1.0
TURN_DURATION = 1.0
WALL_THRESHOLD = 0.35  # Abstand, ab dem eine Wand "da" ist

# --- Bewegung ---
def drive_forward(client, duration=0.25):
    print("⬆️ Geradeaus")
    move(client=client, x=BASE_SPEED, y=0, timeout=duration)

def turn_left(client):
    print("↪️ Links abbiegen")
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION)
    #drive_forward(client, duration=0.2)

def turn_right(client):
    print("↩️ Rechts abbiegen")
    rotate(client=client, speed=-ROTATE_SPEED, timeout=TURN_DURATION)
    drive_forward(client, duration=0.1)

# --- Lidar-Erfassung ---
def get_distances(lidar):
    return {
        # FRONT
        'front_narrow': lidar.get_value_around_angle_min(0, math.pi / 12),   # 15°
        'front_wide':   lidar.get_value_around_angle_min(0, math.pi / 4),    # 45°

        # LEFT
        'left_narrow':  lidar.get_value_around_angle_min(-math.pi / 2, math.pi / 12),
        'left_wide':    lidar.get_value_around_angle_min(-math.pi / 2, math.pi / 4),

        # Extra für Entscheidung
        'light_left':   lidar.get_value_around_angle_min(-math.pi / 4, math.pi / 6),
        'hard_left':    lidar.get_value_around_angle_min(-3 * math.pi / 4, math.pi / 6),
    }

# --- Entscheidungslogik ---
def is_wall(dist_min):
    return dist_min < WALL_THRESHOLD and dist_min > 0.01

def is_left_open(dist):
    # Nimmt das Minimum aus zwei Scans
    left_min = min(dist['left_narrow'], dist['left_wide'], dist['light_left'], dist['hard_left'])
    return not is_wall(left_min)

def is_front_open(dist):
    front_min = min(dist['front_narrow'], dist['front_wide'])
    return not is_wall(front_min)

# --- Hauptlogik ---
def main(client):
    while True:
        lidar = get_lidar_data_once(client, True)
        dist = get_distances(lidar)

        if is_left_open(dist):
            turn_left(client)
        elif is_front_open(dist):
            drive_forward(client)
        else:
            turn_right(client)

        time.sleep(0.05)

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
