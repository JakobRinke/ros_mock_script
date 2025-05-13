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
            break
        print("Current rotation:", rot)
        if rot*math.pi < target_angle:
            dr = 1
        else:
            dr = -1
        rotate(client=client, speed=-speed * dr, timeout=timeout)


def do_a_rotation_of(client:roslibpy.Ros, angle:float) -> None:
    odometry = heiner_comunication.odometry.get_odometry_data_once(client=client)
    start_angle = odometry.orientation.z  # Assume orientation.z is already in radians
    target_angle = start_angle + math.radians(angle)  # Calculate target angle in radians
    speed = 1
    time_interval = 0.1
    angle_diff = angle  # Initial angle difference

    for i in range(300):
        rotate(client=client, speed=speed if angle_diff > 0 else -speed, timeout=time_interval)
        odometry = heiner_comunication.odometry.get_odometry_data_once(client=client)
        current_angle = odometry.orientation.z  # Assume orientation.z is in radians
        angle_diff = (target_angle - current_angle) % (2 * math.pi)  # Normalize to [0, 2*pi)
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi  # Adjust for shortest path in negative direction
        if abs(angle_diff) < 0.02:
            break