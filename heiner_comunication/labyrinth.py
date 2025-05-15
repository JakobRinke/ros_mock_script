from heiner_comunication.lidar import get_lidar_data_once
import roslibpy
import math
import time

ZERO_MESSAGE = {
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
}

LEFT_TURN_CLEAR_THRESHOLD = 0.34  # Meter
FRONT_BLOCKED_THRESHOLD = 0.23  # Meter

CORRECTION_GAIN = 0.6  # Sehr geringe Korrektur
OPEN_CORRIDOR_DISTANCE = 0.4  # Wenn beide Seiten offen -> keine Korrektur

LEFT_WALL_TARGET_DISTANCE = 0.25  # Zielabstand zur linken Wand in Metern

def go_through_corridor_left_wall_follow(client: roslibpy.Ros, base_speed: float, max_duration: float) -> str:
    """
    Linkswandfolge mit adaptiver Korrekturstärke, abhängig von der Enge des Gangs.
    """
    BASE_KP_LEFT = 1.0 * CORRECTION_GAIN
    min_angular = 0.015
    max_angular = 0.4

    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')

    start_time = time.time()

    while time.time() - start_time < max_duration:
        lidar = get_lidar_data_once(client=client)

        front_distance = lidar.get_value_around_angle_min(0, math.radians(15.5))
        left_distance = lidar.get_value_around_angle_min(math.radians(90), math.radians(15.5))

        if front_distance < FRONT_BLOCKED_THRESHOLD:
            talker.publish(ZERO_MESSAGE)
            print("Front blockiert, stoppe im Gang.")
            return 'FRONT_BLOCKED'

        if left_distance > LEFT_TURN_CLEAR_THRESHOLD:
            talker.publish(ZERO_MESSAGE)
            print("Links große Öffnung erkannt, mögliche Entscheidung notwendig.")
            return 'LEFT_OPEN'

        # Nutze den Fokusbereich 85-105 Grad
        left_focus_sector = lidar.get_values_between_angles(math.radians(85), math.radians(105))
        if not left_focus_sector:
            continue  # Sicherstellen dass Daten da sind

        # Dynamische Metrik: Enge und Unruhe bestimmen
        min_dist = min(left_focus_sector)
        max_dist = max(left_focus_sector)
        spread = max_dist - min_dist

        diff_left = min_dist - LEFT_WALL_TARGET_DISTANCE

        # Adaptive Korrektur basierend auf Spread (kleiner Spread -> enger -> höhere Kp)
        # Spread von 0.0m -> 2.0 * BASE_KP_LEFT (sehr eng, sehr präzise)
        # Spread von 0.15m oder größer -> 0.8 * BASE_KP_LEFT (offen, weniger Korrektur)
        if spread < 0.05:
            adaptive_kp = BASE_KP_LEFT * 2.0
        elif spread < 0.1:
            adaptive_kp = BASE_KP_LEFT * 1.5
        elif spread < 0.15:
            adaptive_kp = BASE_KP_LEFT
        else:
            adaptive_kp = BASE_KP_LEFT * 0.8

        # Optional harte Reduktion in breiten Gängen
        if min_dist > OPEN_CORRIDOR_DISTANCE:
            correction_angular = 0.0
        else:
            correction_angular = adaptive_kp * diff_left

            # Deadzone
            if abs(diff_left) < 0.015:
                correction_angular = 0.0

            # Clamp
            correction_angular = max(-max_angular, min(correction_angular, max_angular))

        cmd = {
            'linear': {'x': base_speed, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': correction_angular}
        }

        talker.publish(cmd)
        time.sleep(0.05)

    talker.publish(ZERO_MESSAGE)
    print("Maximale Dauer erreicht, Gang weiter offen.")
    return 'TIMEOUT'
