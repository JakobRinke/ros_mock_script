

import math

# Schwellenwerte
FRONT_OPEN_DISTANCE = 0.2
LEFT_OPEN_DISTANCE = 0.4
LIGHT_LEFT_OPEN_DISTANCE = 0.4
RIGHT_OPEN_DISTANCE = 0.4

def is_front_clear(lidar):
    """
    Prüft ob vor dem Roboter genügend Platz ist.
    Bereich: -22.5° bis +22.5°
    """
    return lidar.get_value_around_angle_min(0, math.radians(22.5)) > FRONT_OPEN_DISTANCE

def is_light_left_clear(lidar):
    """
    Prüft ob leicht links genügend Platz ist.
    Bereich: 22.5° bis 67.5°
    """
    light_left_angle = math.radians(45)
    return lidar.get_value_around_angle_min(light_left_angle, math.radians(22.5)) > LIGHT_LEFT_OPEN_DISTANCE

def is_left_clear(lidar):
    """
    Prüft ob links genügend Platz ist.
    Bereich: 67.5° bis 112.5°
    """
    left_angle = math.radians(90)
    return lidar.get_value_around_angle_min(left_angle, math.radians(22.5)) > LEFT_OPEN_DISTANCE

def is_right_clear(lidar):
    """
    Prüft ob rechts genügend Platz ist.
    Bereich: -67.5° bis -112.5°
    """
    right_angle = -math.radians(90)
    return lidar.get_value_around_angle_min(right_angle, math.radians(22.5)) > RIGHT_OPEN_DISTANCE




