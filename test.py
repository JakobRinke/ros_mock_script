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
    # lidar = get_lidar_data_once(client=client)
    

    # print(lidar.get_value_around_angle(0, math.pi / 8))

    #move(client=client, x=0.1, y=0)
    #rotate(client=client, speed=4)  

    odo = get_odometry_data_once(client=client)
    print(odo)

    time.sleep(1)  


    client.terminate()