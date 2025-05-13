import math
import roslibpy
import time
import heiner_comunication.odometry

def move(client:roslibpy.Ros, x:float, y:float, timeout:float=1) -> None:
    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')

    # Erstelle die Bewegung: Vorwärts
    forward_message = {
        'linear': {'x': x, 'y': y, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }
    talker.publish(forward_message)
    time.sleep(timeout)
    talker.publish(ZERO_MESSAGE)

ZERO_MESSAGE = {
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
}

def rotate(client:roslibpy.Ros, speed:float, timeout:float=1) -> None:
    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')

    # Erstelle die Bewegung: Drehen
    rotate_message = {
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': speed}
    }
    talker.publish(rotate_message)
    time.sleep(timeout)
    talker.publish(ZERO_MESSAGE)



def rotate_to_angle(client:roslibpy.Ros, target_angle:float, speed:float=1, accuracy=0.1) -> None:
    timeout = 0.1
    for i in range(1000):
        odometry = heiner_comunication.odometry.get_odometry_data_once(client=client)
        rot = odometry.orientation.z
        if abs(rot*math.pi - target_angle) < accuracy:
            print("Target angle reached")
            talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
            talker.publish(ZERO_MESSAGE)  # Ensure the robot stops completely
            break
        print("Current rotation:", rot)
        if rot*math.pi < target_angle:
            dr = 1
        else:
            dr = -1
        rotate(client=client, speed=-speed * dr, timeout=timeout)

def sign(x:float) -> int:
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0
    

import math
import roslibpy

ZERO_MESSAGE = {
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
}

def sign(x):
    return 1 if x >= 0 else -1

def normalize_angle(angle):
    """Normalisiere den Winkel auf den Bereich [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

import math
import roslibpy
import time

ZERO_MESSAGE = {
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
}

def sign(x):
    """Bestimmt das Vorzeichen eines Wertes."""
    return 1 if x >= 0 else -1

def normalize_angle(angle):
    """Normalisiere den Winkel auf den Bereich [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

import math
import roslibpy
import time

ZERO_MESSAGE = {
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
}

def sign(x):
    """Bestimmt das Vorzeichen eines Wertes."""
    return 1 if x >= 0 else -1

def normalize_angle(angle):
    """Normalisiere den Winkel auf den Bereich [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi



## THIS FUNCTION DOES NOT WORK.
def do_a_rotation_of(client: roslibpy.Ros, angle: float) -> None:
    # Umrechnung der Eingabewinkel in den Bereich [-pi, pi]
    target_angle = normalize_angle(angle)

    speed = 0.6  # Rotationsgeschwindigkeit
    
    # Holen der Odometriedaten
    odometry = heiner_comunication.odometry.get_odometry_data_once(client=client)
    start_angle = odometry.orientation.z * math.pi  # Wenn die Odometrie den Winkel im Bereich [-1, 1] gibt
    
    # Normalisieren des Startwinkels
    start_angle = normalize_angle(start_angle)

    talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
    
    # Berechnung des Zielwinkels relativ zum Startwinkel
    target_angle = normalize_angle(start_angle + target_angle)
    
    # Berechnung der Differenz zwischen Start- und Zielwinkel
    angle_diff = target_angle - start_angle

    # Normalisierung der Differenz auf den Bereich [-pi, pi]
    angle_diff = normalize_angle(angle_diff)

    # Bestimmen der Drehrichtung basierend auf der Differenz
    direction = sign(angle_diff)

    # Start der Rotation
    talker.publish({
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': direction * speed}
    })

    # Rotationsschleife
    while True:
        # Holen der aktuellen Odometriedaten
        odometry = heiner_comunication.odometry.get_odometry_data_once(client=client)
        current_angle = odometry.orientation.z * math.pi  # Angenommen, orientierung.z gibt den aktuellen Winkel

        # Berechnung der aktuellen Differenz zum Zielwinkel
        angle_diff = current_angle - target_angle

        # Normalisierung der Differenz auf den Bereich [-pi, pi]
        angle_diff = normalize_angle(angle_diff)

        # Wenn die Differenz klein genug ist, stoppen wir
        if abs(angle_diff) < 0.1:
            break

        # Kurze Pause, um CPU-Belastung zu verringern
        time.sleep(0.1)

    # Stoppen des Roboters nach Erreichen des Zielwinkels
    talker.publish(ZERO_MESSAGE)
