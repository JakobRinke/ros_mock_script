import roslibpy
import math
import time
from heiner_comunication.lidar import LidarFrame, get_lidar_data_once
from heiner_comunication.motor_control import move, rotate

# Placeholder-Funktionen für den Magnetfeld-Check und Ultraschall (False-Ausgabe)
def check_for_magnet_field():
    return False

def ultrasonic_infront_wall() -> bool:
    return False

# --- Konfiguration ---
BASE_SPEED = 0.6
FORWARD_SPEED_WHEN_ROTATING = 0.2

DETECT_WALL_THRESHOLD = 0.4  # Höher ansetzen, damit Wände früher erkannt werden
LIDAR_FREE_THRESHOLD = 1.5    # Erkennen von "offenen" Stellen robuster
PREFERED_RIGHT_DISTANCE = 0.25  # Etwas mehr Abstand für weichere Korrektur
MAX_ANGULAR = 2.0
MIN_ANGULAR = 0.3

def lidar_infront_wall(lidar_data: LidarFrame) -> bool:
    return lidar_data.get_value_around_angle(0, math.pi / 8) < DETECT_WALL_THRESHOLD

def lidar_all_free(lidar_data: LidarFrame) -> bool:
    visions = [
        lidar_data.get_value_around_angle(0, math.pi / 8),
        lidar_data.get_value_around_angle(-math.pi / 4, math.pi / 8),
        lidar_data.get_value_around_angle(math.pi / 4, math.pi / 8),
        lidar_data.get_value_around_angle(-math.pi / 2, math.pi / 8),
        lidar_data.get_value_around_angle(math.pi / 2, math.pi / 8)
    ]
    return sum(vision > LIDAR_FREE_THRESHOLD for vision in visions) >= 4

def correct_left_wall_distance(lidar_data: LidarFrame) -> float:
    left_distance = lidar_data.get_value_around_angle(-math.pi / 2, math.pi / 6)
    right_distance = lidar_data.get_value_around_angle(math.pi / 2, math.pi / 6)
    return left_distance - right_distance

def is_there_a_wall_on_the_left(lidar_data: LidarFrame) -> bool:
    left_distance = lidar_data.get_value_around_angle(-math.pi / 2, math.pi / 5)
    return left_distance < DETECT_WALL_THRESHOLD

def is_dead_end(lidar_data: LidarFrame) -> bool:
    front = lidar_data.get_value_around_angle(0, math.pi / 8)
    left = lidar_data.get_value_around_angle(-math.pi / 2, math.pi / 8)
    right = lidar_data.get_value_around_angle(math.pi / 2, math.pi / 8)
    return (
        front < DETECT_WALL_THRESHOLD + 0.1 and
        left < DETECT_WALL_THRESHOLD + 0.1 and
        right < DETECT_WALL_THRESHOLD + 0.1
    )

def is_narrow_corridor(lidar_data: LidarFrame) -> bool:
    left = lidar_data.get_value_around_angle(-math.pi / 2, math.pi / 8)
    right = lidar_data.get_value_around_angle(math.pi / 2, math.pi / 8)
    return left < 0.4 and right < 0.4


def is_front_blocked(lidar_data: LidarFrame) -> bool:
    front_distance = lidar_data.get_value_around_angle(0, math.pi / 8)
    return front_distance < DETECT_WALL_THRESHOLD


def hard_escape_until_clear(client: roslibpy.Ros):
    print("? HARTE ESCAPE (Dead End Erkennung)")
    timeout = 0
    while timeout < 5:
        lidar_data = get_lidar_data_once(client=client)
        if not is_front_blocked(lidar_data):
            print("?? Front wieder frei nach harter Drehung")
            break
        rotate(client=client, speed=1, timeout=0.5)
        move(client=client, x=-FORWARD_SPEED_WHEN_ROTATING, y=0, timeout=0.3)
        timeout += 0.5

def is_narrow_corridor(lidar_data: LidarFrame) -> bool:
    left = lidar_data.get_value_around_angle(-math.pi / 2, math.pi / 8)
    right = lidar_data.get_value_around_angle(math.pi / 2, math.pi / 8)
    # Erhöhe den Schwellenwert, um nicht sofort einen normalen schmalen Gang als "narrow" zu interpretieren
    return left < 0.3 and right < 0.3

def go_forward_a_bit_and_keep_right(client: roslibpy.Ros, talker: roslibpy.Topic, lidar_data: LidarFrame, correction_boost=1.0) -> None:
    side_right = lidar_data.get_value_around_angle(-math.pi / 2, math.pi / 8)
    diag_right = lidar_data.get_value_around_angle(-math.pi / 3, math.pi / 8)
    front_right = lidar_data.get_value_around_angle(-math.pi / 6, math.pi / 8)

    error_side = side_right - PREFERED_RIGHT_DISTANCE
    error_diag = diag_right - (PREFERED_RIGHT_DISTANCE + 0.1)
    error_front_right = front_right - (PREFERED_RIGHT_DISTANCE + 0.2)

    correction_angular = (error_side * 1.0 + error_diag * 1.5 + error_front_right * 2.5) / 5.0
    correction_angular *= correction_boost
    correction_angular = max(-MAX_ANGULAR, min(correction_angular, MAX_ANGULAR))

    forward_speed = BASE_SPEED * (1.0 - min(abs(correction_angular) * 0.5, 0.7))
    forward_speed = max(0.1, forward_speed)

    cmd = {
        'linear': {'x': forward_speed, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': -correction_angular}
    }

    talker.publish(cmd)

def main(client: roslibpy.Ros):
    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
    state = 'CHECK_GOAL'

    while True:
        lidar_data = get_lidar_data_once(client=client)

        # Check for a magnet field - if there is one, we found the goal
        if check_for_magnet_field():
            print("Found magnet field, goal reached!")
            break
        
        if state == 'CHECK_GOAL':
            if is_dead_end(lidar_data):
                print("🛑 Dead End erkannt -> HARD_ESCAPE")
                state = 'HARD_ESCAPE'
            elif is_front_blocked(lidar_data):
                print("⚠ Front blockiert -> ESCAPE")
                state = 'ESCAPE'
            elif not is_there_a_wall_on_the_left(lidar_data):
                print("⬅ Kein Wand links -> TURN LEFT")
                state = 'TURN_LEFT'
            elif is_narrow_corridor(lidar_data):
                print("↔ Enger Gang erkannt -> Weichere Korrektur")
                state = 'FOLLOW_WALL'
            else:
                state = 'FOLLOW_WALL'

        elif state == 'HARD_ESCAPE':
            hard_escape_until_clear(client=client)
            state = 'CHECK_GOAL'

        elif state == 'FOLLOW_WALL':
            if is_dead_end(lidar_data):
                print("🛑 Dead End erkannt während FOLLOW_WALL -> HARD_ESCAPE")
                state = 'HARD_ESCAPE'
            elif is_front_blocked(lidar_data):
                print("⚠ Front blockiert während FOLLOW_WALL -> ESCAPE")
                state = 'ESCAPE'
            elif not is_there_a_wall_on_the_left(lidar_data):
                print("⬅ Wand verloren während FOLLOW_WALL -> TURN LEFT")
                state = 'TURN_LEFT'
            else:
                # Weiche Korrektur in engen Gängen
                go_forward_a_bit_and_keep_right(client=client, talker=talker, lidar_data=lidar_data)

        elif state == 'ESCAPE':
            print("🚶‍♂️ Escape Logik -> Rückwärts und drehen")
            rotate(client=client, speed=1, timeout=2)
            move(client=client, x=-FORWARD_SPEED_WHEN_ROTATING, y=0, timeout=1)
            state = 'CHECK_GOAL'

        elif state == 'TURN_LEFT':
            rotate(client=client, speed=0.9, timeout=1)
            move(client=client, x=FORWARD_SPEED_WHEN_ROTATING, y=0, timeout=0.5)
            state = 'CHECK_GOAL'



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
        time.sleep(1)
        client.terminate()
