import roslibpy
import math
import time
from heiner_comunication.lidar import get_lidar_data_once, LidarFrame
from heiner_comunication.motor_control import move, rotate

# Parameter
FORWARD_SPEED = 1.0  # Geschwindigkeit beim Vorwärtsfahren
ROTATE_SPEED = 0.6   # Geschwindigkeit beim Drehen
FORWARD_DURATION = 0.5  # Dauer für kleine Vorwärtsbewegung
TURN_DURATION = 1.0     # Dauer für kleine Drehung

# Schwellwerte
FRONT_ANGLE = 0
FRONT_SPREAD = math.radians(30)  # +-30 Grad
RIGHT_ANGLE = 3 * math.pi / 2
RIGHT_SPREAD = math.radians(30)  # +-30 Grad
DISTANCE_THRESHOLD = 0.24  # Mindestabstand in Meter
RIGHT_OPEN_THRESHOLD = 0.3  # Mehr Abstand für rechts nötig

# Kreisfahrt-Erkennung
MAX_RIGHT_TURNS = 3

def get_distance(lidar:LidarFrame, angle, spread):
    """Gibt den durchschnittlichen Abstand um einen bestimmten Winkel."""
    return lidar.get_value_around_angle(angle, spread)


def is_front_clear(lidar):
    distance = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
    return distance > DISTANCE_THRESHOLD


def is_right_clear(lidar):
    distance = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
    return distance > RIGHT_OPEN_THRESHOLD


def get_right_opening_angle(lidar: LidarFrame):
    """Bestimme, wie weit der freie Raum rechts offen ist."""
    start_angle = RIGHT_ANGLE - math.radians(90)
    stop_angle = RIGHT_ANGLE + math.radians(30)
    scan_data = lidar.get_values_between_angles(start_angle, stop_angle)

    max_distance = 0
    best_angle = RIGHT_ANGLE

    if len(scan_data) == 0:
        return -math.radians(45)  # Fallback falls keine Daten vorhanden

    for i, distance in enumerate(scan_data):
        if distance > max_distance:
            max_distance = distance
            # Berechne den aktuellen Winkel exakt
            angle = start_angle + (stop_angle - start_angle) * i / len(scan_data)
            best_angle = angle

    # Winkel relativ zur Front normalisieren (-pi bis +pi)
    angle_diff = (best_angle - FRONT_ANGLE + math.pi) % (2 * math.pi) - math.pi
    return angle_diff



def move_forward(client):
    move(client, FORWARD_SPEED, 0, FORWARD_DURATION)


def move_forward_a_bit(client):
    move(client, FORWARD_SPEED * 0.2, 0, FORWARD_DURATION * 0.6)

def move_backward(client):
    """Bewege den Roboter rückwärts."""
    move(client, -FORWARD_SPEED, 0, FORWARD_DURATION)

def move_backward_a_bit(client):
    """Bewege den Roboter ein kleines Stück rückwärts."""
    move(client, -FORWARD_SPEED * 0.15, 0, FORWARD_DURATION * 0.4)


def rotate_right(client):
    rotate(client=client, speed=-ROTATE_SPEED, timeout=TURN_DURATION)


def rotate_left(client):
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION)

def rotate_left_agressive(client):
    """Aggressive Linksdrehung."""
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION * 2.3)


def cautious_right_turn(client):
    """Mehrstufige vorsichtige Rechtsdrehung mit kleinen Bewegungen."""
    for _ in range(3):
        rotate(client=client, speed=-ROTATE_SPEED, timeout=0.4)
        move_forward_a_bit(client)
        lidar = get_lidar_data_once(client, True)
        if not is_right_clear(lidar):
            break


def wall_follower(client):
    """Verbesserter Right-Hand Wall Follower Algorithmus."""
    right_turn_counter = 0

    while True:
        lidar = get_lidar_data_once(client, True)

        if is_right_clear(lidar):
            right_turn_counter += 1

            right_angle = get_right_opening_angle(lidar)
            limited_angle = max(-math.radians(60), min(right_angle, -math.radians(10)))

            print(f"🔄 Rechts offen, drehe ca. {math.degrees(limited_angle):.1f}°")
            rotate(client=client, speed=-ROTATE_SPEED, timeout=abs(limited_angle) / (ROTATE_SPEED * 2))
            new_lidar = get_lidar_data_once(client, True)
            if is_front_clear(new_lidar):
                move_forward_a_bit(client)
            else:
                print("🚧 Vorne blockiert, mache vorsichtige Rückwärtbewegung")
                move_backward_a_bit(client)

        elif is_front_clear(lidar):
            right_turn_counter = 0
            move_forward(client)

        else:
            right_turn_counter = 0
            rotate_left(client)

        if right_turn_counter > MAX_RIGHT_TURNS:
            print("🚨 Mögliche Kreisfahrt erkannt – aggressive Linksdrehung!")
            move(client, -FORWARD_SPEED, 0, FORWARD_DURATION / 1.2)
            rotate_left_agressive(client)
            move_forward_a_bit(client)
            time.sleep(1.5)  # Lange Linksdrehung zum Rauskommen
            right_turn_counter = 0

        time.sleep(0.05)


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
