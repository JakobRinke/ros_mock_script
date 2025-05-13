from heiner_comunication.lidar import LidarFrame, get_lidar_data_once
from heiner_comunication.motor_control import move, rotate
import roslibpy
import math
import time

ZERO_MESSAGE = {
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
}

def go_through_corridor_center(client: roslibpy.Ros, base_speed: float, duration: float) -> None:
    """
    Fährt mittig durch den Korridor und nutzt zusätzliche Front-Schrägwinkel zur Korrektur.
    """
    Kp_side = 2.5
    Kp_front = 1.5
    min_angular = 0.2
    max_angular = 1.5

    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')

    start_time = time.time()

    while time.time() - start_time < duration:
        lidar = get_lidar_data_once(client=client)

        # Seitenmessung
        left_distance = lidar.get_value_around_angle(-math.pi / 2, math.pi / 5)
        right_distance = lidar.get_value_around_angle(math.pi / 2, math.pi / 5)
        diff_side = left_distance - right_distance

        # Schräg-Front-Messung
        front_left = lidar.get_value_around_angle(-math.pi / 4, math.pi / 8)
        front_right = lidar.get_value_around_angle(math.pi / 4, math.pi / 8)
        diff_front = front_left - front_right

        # Kombinierte Korrektur (Seite dominant, Front unterstützend)
        correction_angular = (Kp_side * diff_side) + (Kp_front * diff_front)
        correction_angular = max(-max_angular, min(correction_angular, max_angular))

        # Dynamische Geschwindigkeit
        speed = base_speed * (1.0 - min(abs(diff_side), 1.0))
        speed = max(0.05, speed)

        cmd = {
            'linear': {'x': speed, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': -correction_angular}
        }

        talker.publish(cmd)
        time.sleep(0.05)

    talker.publish(ZERO_MESSAGE)
