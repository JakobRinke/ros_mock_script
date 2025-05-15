import roslibpy
import math
import time
from heiner_comunication.lidar import get_lidar_data_once
from heiner_comunication.motor_control import move, rotate

# --- Konfiguration ---
BASE_SPEED = 0.2
ROTATE_SPEED = 1.0
TURN_DURATION = 0.55
WALL_THRESHOLD_FRONT = 0.4
WALL_THRESHOLD_SIDE = 0.3
PREFERRED_LEFT_DISTANCE = 0.25
ANGLE_CORRECTION_FACTOR = 0.5
CORRECTION_EVERY_N_STEPS = 3
MAX_VALID_LEFT_DISTANCE = 0.6  # Neu: maximale akzeptierte Linkswand-Distanz

# --- Bewegung ---
def drive_forward(client, duration=0.25):
    print("⬆️ Geradeaus")
    move(client=client, x=BASE_SPEED, y=0, timeout=duration)

def turn_left(client):
    print("↪️ Links abbiegen")
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION)

def angle_correction(client, delta):
    angle = max(min(delta, 0.2), -0.2) * ANGLE_CORRECTION_FACTOR
    print(f"🔧 Y Korrektur: {angle:.3f}")
    move(client=client, x=0, y=angle, timeout=0.2)
    rotate(client=client, speed=ROTATE_SPEED, timeout=0.3)

# --- Lidar-Auswertung ---
def get_distances(lidar):
    return {
        'front': lidar.get_value_around_angle_min(0, math.pi / 8),     # 30°
        'left':  lidar.get_value_around_angle_min(math.pi / 2, math.pi / 5),  # 30°
    }

def is_wall(dist, threshold):
    return dist < threshold and dist > 0.01

# --- Hauptlogik ---
def main(client):
    step = 0
    left_turns_in_a_row = 0

    while True:
        lidar = get_lidar_data_once(client, True)
        dist = get_distances(lidar)

        front_blocked = is_wall(dist['front'], WALL_THRESHOLD_FRONT)

        # Linkswand verloren?
        if dist['left'] > MAX_VALID_LEFT_DISTANCE or dist['left'] < 0.01:
            print("🔄 Wand links verloren, leichte Linksdrehung zur Neuorientierung")
            rotate(client=client, speed=ROTATE_SPEED * 0.5, timeout=0.3)
            continue

        left_open = not is_wall(dist['left'], WALL_THRESHOLD_SIDE)

        if left_open and left_turns_in_a_row < 3:
            turn_left(client)
            left_turns_in_a_row += 1
            if left_turns_in_a_row == 3:
                drive_forward(client)

        elif not front_blocked:
            if step % CORRECTION_EVERY_N_STEPS == 0:
                delta = dist['left'] - PREFERRED_LEFT_DISTANCE
                if abs(delta) < 0.3:
                    angle_correction(client, delta)
            drive_forward(client)
            left_turns_in_a_row = 0
            step += 1

        else:
            turn_left(client)
            left_turns_in_a_row += 1
            if left_turns_in_a_row == 3:
                drive_forward(client)

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
