import roslibpy
import math
import time
from heiner_comunication.lidar import LidarFrame, get_lidar_data_once
from heiner_comunication.motor_control import move, rotate

# --- Konfiguration ---
BASE_SPEED = 0.5
ROTATE_SPEED = 0.9
ESCAPE_SPEED = 0.25
MAX_ANGULAR = 2.0
WALL_DETECT_THRESHOLD = 0.4
PREFERRED_RIGHT_DISTANCE = 0.25

# --- Lidar-Abfragen ---
def get_distances(lidar: LidarFrame):
    return {
        'front': lidar.get_value_around_angle(0, math.pi / 8),
        'left': lidar.get_value_around_angle(-math.pi / 2, math.pi / 8),
        'right': lidar.get_value_around_angle(math.pi / 2, math.pi / 8)
    }

def is_dead_end(distances):
    print(f"[Dead-End Check] front: {distances['front']:.2f}, left: {distances['left']:.2f}, right: {distances['right']:.2f}")
    return all(d < WALL_DETECT_THRESHOLD + 0.1 for d in distances.values())

# --- Bewegung ---
def correct_right_wall(client, talker, lidar):
    side = lidar.get_value_around_angle(-math.pi / 2, math.pi / 8)
    diag = lidar.get_value_around_angle(-math.pi / 3, math.pi / 8)
    front = lidar.get_value_around_angle(-math.pi / 6, math.pi / 8)

    err_side = side - PREFERRED_RIGHT_DISTANCE
    err_diag = diag - (PREFERRED_RIGHT_DISTANCE + 0.1)
    err_front = front - (PREFERRED_RIGHT_DISTANCE + 0.2)

    angular_correction = (err_side + 1.5 * err_diag + 2.5 * err_front) / 5
    angular_correction = max(-MAX_ANGULAR, min(angular_correction, MAX_ANGULAR))

    linear_speed = max(0.1, BASE_SPEED * (1.0 - min(abs(angular_correction) * 0.5, 0.7)))

    cmd = {
        'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': -angular_correction}
    }
    talker.publish(cmd)

def rotate_left(client, duration=1.0):
    print("↪️ Drehe nach links")
    rotate(client=client, speed=ROTATE_SPEED, timeout=duration)

def rotate_in_place(client):
    print("? ESCAPE: Drehen in Sackgasse")
    rotate(client=client, speed=ROTATE_SPEED, timeout=1.5)

# --- Hauptlogik ---
def main(client: roslibpy.Ros):
    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
    stuck_counter = 0

    while True:
        lidar = get_lidar_data_once(client, True)
        distances = get_distances(lidar)

        if is_dead_end(distances):
            stuck_counter += 1
            if stuck_counter >= 3:
                print("⚠️ Anti-Stuck: Spezialmanöver")
                move(client=client, x=0.3, y=0, timeout=1.0)
                rotate(client=client, speed=ROTATE_SPEED, timeout=2.0)
                stuck_counter = 0
            else:
                rotate_in_place(client)
            continue

        # Reset stuck counter if normal
        stuck_counter = 0

        if distances['front'] < WALL_DETECT_THRESHOLD:
            # Front blockiert → drehe links
            rotate_left(client, duration=1.0)
            continue

        if distances['left'] > 0.6:
            # Freie linke Seite → folge Wand
            rotate_left(client, duration=1.0)
            continue

        # Sonst: geradeaus und rechts korrigieren
        correct_right_wall(client, talker, lidar)


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
