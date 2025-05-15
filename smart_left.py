import math
import time
from heiner_comunication.lidar_signature import *
from heiner_comunication.motor_control import rotate, move
from heiner_comunication.smart_rotation import *
import roslibpy
from heiner_comunication.lidar import get_lidar_data_once
from heiner_comunication.motor_control import move, rotate
from heiner_comunication.labyrinth import go_through_corridor_left_wall_follow

ROTATION_TIME = 0.2  # Zeit in Sekunden für eine 90° Drehung
ROTATE_SPEED = 0.8  # Setze passende Geschwindigkeit
MATCH_THRESHOLD = 0.1
TIMEOUT_SEC = 10  # Sicherheits-Timeout

def rotate_left_until_signature_matches(client, target_signature):
    """
    Dreht den Roboter schrittweise nach links, bis die um -2 Schritte (90°) gedrehte
    Signatur möglichst ähnlich der aktuellen Signatur ist.
    """
    start_time = time.time()
    best_difference = float('inf')
    current_threshold = MATCH_THRESHOLD
    while True:
        # Neue Signatur nach Drehung aufnehmen
        lidar = get_lidar_data_once(client, True)
        current_signature = get_lidar_signature(lidar)

        # Vergleichen
        difference = compare_signatures(current_signature, target_signature)
        best_difference = min(best_difference, difference)

        print(f"Signatur Differenz: {difference:.2f}")

        if difference < current_threshold:
            print("Ziel erreicht: Signatur übereinstimmend.")
            break

        # Drehen
        rotate(client=client, speed=ROTATE_SPEED, timeout=ROTATION_TIME)
        
        # Timeout
        if time.time() - start_time > TIMEOUT_SEC:
            print("Timeout erreicht, kein Matching. Reduziere Genauigkeit und starte neu.")
            current_threshold *= 1.5  # Genauigkeit reduzieren
            start_time = time.time()  # Timeout zurücksetzen
            continue

    # Dreh dich noch einmal nach links um zu testen ob die Signatur dadurch noch besser wird, wenn nicht dann drehe wieder zurück
    rotate(client=client, speed=ROTATE_SPEED, timeout=ROTATION_TIME)
    lidar = get_lidar_data_once(client, True)
    current_signature = get_lidar_signature(lidar)
    difference = compare_signatures(current_signature, target_signature)
    if difference > best_difference:
        print("Signatur hat sich verschlechtert, Rückdrehung.")
        rotate(client=client, speed=-ROTATE_SPEED, timeout=ROTATION_TIME)

    print(f"Signatur Differenz nach Rückdrehung: {difference:.2f}")
    

def rotate_in_place(client, degrees: float):
    n = int(degrees / 45)
    lidar = get_lidar_data_once(client, True)
    signature = get_lidar_signature(lidar)
    target_signature = rotate_signature(signature, n)
    rotate_left_until_signature_matches(client, target_signature)



def main(client: roslibpy.Ros):
    while True:
        # 1. Korridor folgen
        result = go_through_corridor_left_wall_follow(client, base_speed=0.15, max_duration=20.0)

        # 2. Nach dem Stop nochmal sicherheitshalber den Lidar checken
        lidar = get_lidar_data_once(client)

        # 3. Links offen? ? Linksdrehung & bisschen vorfahren
        if is_left_clear(lidar) or result == 'LEFT_OPEN':
            print("Linksöffnung bestätigt (nach Lidar-Check). Starte Linksdrehung.")
            # go a bit forward
            move(client, x=0.2, y=0, timeout=0.2)
            rotate_in_place(client, degrees=90)
            move(client, x=0.3, y=0, timeout=1.5)
            # Nach vorne fahren bis links blockiert (optional, kann auch über dein bestehendes Modul passieren)
            continue

        # 4. Front blockiert, aber kein Links ? 180 Grad drehen
        elif not is_front_clear(lidar):
            print("Front blockiert, keine Linksöffnung. Starte 180 Grad Drehung.")
            rotate_in_place(client, degrees=180)
            continue

        # 5. Ansonsten wieder normal weiter
        else:
            print("Nichts blockiert oder offen, fahre weiter.")
            continue

    
# --- Einstiegspunkt ---
if __name__ == "__main__":
    client = roslibpy.Ros(host='192.168.149.1', port=9091)
    try:
        client.run()
        if client.is_connected:
            print("✅ Verbindung zu ROSBridge erfolgreich")
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
