import roslibpy
import time

def move_robot():
    # Verbinde mit dem ROS-Master
    client = roslibpy.Ros(host='localhost', port=11311)
    client.run()

    # Erstelle einen Publisher für das cmd_vel-Topic
    cmd_vel = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')

    # Erstelle eine Nachricht
    twist = roslibpy.Message({
        'linear': {'x': 0.5, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    })

    # Sende die Nachricht
    cmd_vel.publish(twist)
    print("Bewege den Roboter vorwärts.")

    # Stoppe den Roboter nach 2 Sekunden
    time.sleep(2)
    twist['linear']['x'] = 0.0
    cmd_vel.publish(twist)
    print("Roboter gestoppt.")

    # Schließe die Verbindung
    client.terminate()

if __name__ == '__main__':
    move_robot()
