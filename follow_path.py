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

        # Wenn die Front blockiert ist, drehe dich solange, bis sie frei ist
        front_distance = lidar.get_value_around_angle(0, math.pi / 4)
        if front_distance < 0.3:
            talker.publish(ZERO_MESSAGE)
            print("Front blockiert, stoppe im Gang.")
            time.sleep(0.5)
            talker.publish({
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.7}
            })
            while front_distance < 0.3:
                # Drehe dich um 90 Grad nach links
                rotate(client, speed=0.2, timeout=1.5)
                time.sleep(0.5)
                front_distance = lidar.get_value_around_angle(0, math.pi / 4)
            talker.publish(ZERO_MESSAGE)
            time.sleep(0.5)
            return 'FRONT_BLOCKED'
        
       


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


def main(client: roslibpy.Ros):
    """
    Hauptfunktion, die den Korridor folgt.
    """
    print("🚀 Starte Korridorverfolgung...")
    go_through_corridor_center(client, base_speed=0.15, duration=120.0)
    print("✅ Korridorverfolgung abgeschlossen.")


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
