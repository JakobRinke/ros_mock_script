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
    

 
    move(client, 0, 1, timeout=0.2)

   

    client.terminate()