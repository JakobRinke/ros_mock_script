import roslibpy
import math
import time
from heiner_comunication.lidar import LidarFrame, get_lidar_data_once
from heiner_comunication.motor_control import move, rotate

# --- Konfiguration ---
BASE_SPEED = 0.6
ROTATE_SPEED = 0.9
ESCAPE_SPEED = 0.2

WALL_DETECT_THRESHOLD = 0.4
PREFERRED_RIGHT_DISTANCE = 0.25
MAX_ANGULAR = 2.0

# --- Platzhalter für Sensorlogik ---
def check_for_magnet_field() -> bool:
    return False

def ultrasonic_infront_wall() -> bool:
    return False

# --- Hilfsfunktionen ---
def is_dead_end(distances: dict) -> bool:
    front, left, right = distances['front'], distances['left'], distances['right']
    print(f"[Dead-End Check] front: {front:.2f}, left: {left:.2f}, right: {right:.2f}")

    if front > WALL_DETECT_THRESHOLD + 0.15:
        return False  # vorne frei → kein Dead-End

    blocked_sides = sum(d < WALL_DETECT_THRESHOLD + 0.1 for d in [left, right])
    return blocked_sides >= 2


def get_distances(lidar: LidarFrame) -> dict:
    return {
        'front': lidar.get_value_around_angle(0, math.pi / 8),
        'left': lidar.get_value_around_angle(-math.pi / 2, math.pi / 8),
        'right': lidar.get_value_around_angle(math.pi / 2, math.pi / 8)
    }

def is_narrow_corridor(lidar: LidarFrame) -> bool:
    left = lidar.get_value_around_angle(-math.pi / 2, math.pi / 8)
    right = lidar.get_value_around_angle(math.pi / 2, math.pi / 8)
    return left < 0.4 and right < 0.4

def correct_right_wall(client, talker, lidar, boost=1.0):
    side = lidar.get_value_around_angle(-math.pi / 2, math.pi / 8)
    diag = lidar.get_value_around_angle(-math.pi / 3, math.pi / 8)
    front = lidar.get_value_around_angle(-math.pi / 6, math.pi / 8)

    err_side = side - PREFERRED_RIGHT_DISTANCE
    err_diag = diag - (PREFERRED_RIGHT_DISTANCE + 0.1)
    err_front = front - (PREFERRED_RIGHT_DISTANCE + 0.2)

    angular_correction = (err_side + 1.5 * err_diag + 2.5 * err_front) / 5
    angular_correction = max(-MAX_ANGULAR, min(angular_correction * boost, MAX_ANGULAR))

    linear_speed = max(0.1, BASE_SPEED * (1.0 - min(abs(angular_correction) * 0.5, 0.7)))

    cmd = {
        'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': -angular_correction}
    }
    talker.publish(cmd)

def rotate_left(client, duration=1.0):
    print("↪️ Drehe nach links (freie linke Seite)")
    rotate(client=client, speed=ROTATE_SPEED, timeout=duration)

def rotate_in_place(client):
    print("? ESCAPE: Nur Drehen (kein Rückwärtsfahren)")
    rotate(client=client, speed=ROTATE_SPEED, timeout=2)

# --- Hauptlogik ---
def main(client: roslibpy.Ros):
    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
    stuck_counter = 0

    while True:
        lidar = get_lidar_data_once(client)
        distances = get_distances(lidar)

        if check_for_magnet_field():
            print("🏁 Ziel erreicht! (Magnetfeld erkannt)")
            break

        if is_dead_end(distances):
            stuck_counter += 1
            if stuck_counter >= 3:
                print("⚠️ Anti-Stuck: Spezialmanöver bei mehrfachem Steckenbleiben")
                move(client=client, x=0.3, y=0, timeout=1.0)
                rotate(client=client, speed=ROTATE_SPEED, timeout=2.0)
                stuck_counter = 0
            else:
                print("? HARTE ESCAPE aktiviert (echte Sackgasse)")
                rotate_in_place(client)
            continue

        stuck_counter = 0  # Reset bei normalem Verlauf

        # ✅ Geradeaus hat Vorrang
        if distances['front'] > WALL_DETECT_THRESHOLD + 0.1:
            boost = 2.0 if is_narrow_corridor(lidar) else 1.0
            correct_right_wall(client, talker, lidar, boost=boost)
            continue

        # ⬅️ Links frei – abbiegen
        if distances['left'] > 0.6:
            rotate_left(client, duration=1.0)
            continue

        # ❌ Nichts geht – minimal drehen
        rotate_in_place(client)


import time
if __name__ == "__main__":
    client = roslibpy.Ros(host='192.168.149.1', port=9091)
    try:
        client.run()
        if client.is_connected:
            main(client=client)
            client.terminate()
        else:
            print("Could not connect to ROSBridge")
    except KeyboardInterrupt:
        print("Stopping due to keyboard interrupt")
        talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
        stop_cmd = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        talker.publish(stop_cmd)
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            pass
        client.terminate()
