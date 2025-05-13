from heiner_comunication.odometry import *
from heiner_comunication.lidar import *
from heiner_comunication.motor_control import *
import roslibpy
import time
import math


# Verbindung zu ROSBridge auf Port 9091
client = roslibpy.Ros(host='192.168.149.1', port=9091)

client.run()


if client.is_connected:
    do_a_rotation_of(client=client, angle=math.pi / 2)  # Drehe um 90 Grad









    time.sleep(1)  # Warten, um sicherzustellen, dass die Bewegung abgeschlossen ist
    client.terminate()

    