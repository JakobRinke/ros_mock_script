import roslibpy
import math
import time
from heiner_comunication.lidar import get_lidar_data_once
from heiner_comunication.motor_control import move, rotate

# --- Parameter ---
FORWARD_SPEED = 0.3
ROTATE_SPEED = 1.2
FORWARD_AFTER_ROTATE_TIME = 0.7
WALL_THRESHOLD = 0.4         # < als Wand erkannt
OPENING_THRESHOLD = 0.6      # > als Öffnung erkannt

# --- Hauptverhalten: Linkswand folgen ---
def follow_left_wall(client: roslibpy.Ros):
    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')

    def drive_corrected():
        # Lidar-Werte abfragen
        lidar = get_lidar_data_once(client, True)
        left = lidar.get_value_around_angle_min(-math.pi / 2, math.pi / 8)
        front = lidar.get_value_around_angle_min(0, math.pi / 8)
        right = lidar.get_value_around_angle_min(math.pi / 2, math.pi / 8)

        # Fehlerberechnung für sanfte Korrektur
        error = 0.25 - left  # Zielabstand: 25 cm
        angular = max(-0.8, min(0.8, error * 3.0))  # Dynamische Korrektur
        linear = FORWARD_SPEED * (1.0 - min(abs(angular) * 0.5, 0.5))

        # Korrektur der Bewegung
        cmd = {
            'linear': {'x': linear, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': angular}
        }
        talker.publish(cmd)

    def stop():
        cmd = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        talker.publish(cmd)

    while True:
        lidar = get_lidar_data_once(client, True)

        front = lidar.get_value_around_angle_min(0, math.pi / 8)
        left = lidar.get_value_around_angle_min(-math.pi / 2, math.pi / 8)
        right = lidar.get_value_around_angle_min(math.pi / 2, math.pi / 8)

        # 🧱 Sackgasse erkennen (beide Seiten blockiert, aber rechts offen)
        if front < WALL_THRESHOLD and left < WALL_THRESHOLD and right > WALL_THRESHOLD:
            print("🧱 Sackgasse erkannt – drehe nach rechts")
            rotate(client, speed=-ROTATE_SPEED, timeout=1.0)
            move(client, x=FORWARD_SPEED, y=0, timeout=FORWARD_AFTER_ROTATE_TIME)
            continue

        # ↪️ Öffnung links erkannt → links abbiegen
        if left > OPENING_THRESHOLD:
            print("↪️ Öffnung links – drehe ab")
            rotate(client, speed=ROTATE_SPEED, timeout=1.1)
            move(client, x=FORWARD_SPEED, y=0, timeout=FORWARD_AFTER_ROTATE_TIME)
            continue

        # 🧱 Wand vorne → nach links drehen und kurz vorwärts
        if front < WALL_THRESHOLD:
            print("🧱 Blockiert vorne – drehe links")
            rotate(client, speed=ROTATE_SPEED, timeout=1.0)
            move(client, x=FORWARD_SPEED, y=0, timeout=FORWARD_AFTER_ROTATE_TIME)
            continue

        # 🚶 Standardfall: der linken Wand folgen
        print("🚶 Folge linker Wand")
        drive_corrected()
        time.sleep(0.1)


# --- Startpunkt ---
if __name__ == "__main__":
    client = roslibpy.Ros(host='192.168.149.1', port=9091)
    try:
        client.run()
        if client.is_connected:
            follow_left_wall(client)
            client.terminate()
        else:
            print("❌ Verbindung zu ROSBridge fehlgeschlagen")
    except KeyboardInterrupt:
        print("⛔️ Stop durch KeyboardInterrupt")
        talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
        stop_cmd = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        talker.publish(stop_cmd)
        time.sleep(1)
        client.terminate()
