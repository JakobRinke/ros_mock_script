from heiner_comunication.labyrinth import *
import roslibpy
import time
import math


# Verbindung zu ROSBridge auf Port 9091
client = roslibpy.Ros(host='192.168.149.1', port=9091)

client.run()


if client.is_connected:
    
    #go_through_corridor_center(client=client, base_speed=0.3, duration=10)  # Bewege dich durch den Korridor   
    go_through_corridor_left_wall_follow(client, 0.3, 15)  # Bewege dich bis zur Entscheidung






    time.sleep(1)  # Warten, um sicherzustellen, dass die Bewegung abgeschlossen ist
    client.terminate()

    