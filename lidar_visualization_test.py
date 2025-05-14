from heiner_comunication.lidar import *
import roslibpy
import time
import math
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('TkAgg')  # or 'Qt5Agg'

client = roslibpy.Ros(host='192.168.149.1', port=9091)

client.run()

if client.is_connected:
    lidar_data = get_lidar_data_once(client=client)
    client.terminate()

    lidar_points_x = []
    lidar_points_y = []

    for idx, lidar_range in enumerate(lidar_data.ranges):
        angle = lidar_data.angle_min + idx * lidar_data.angle_increment
        lidar_points_x.append(lidar_range * math.cos(angle))
        lidar_points_y.append(lidar_range * math.sin(angle))

    fig, ax = plt.subplots()
    ax.scatter(lidar_points_x, lidar_points_y)
    ax.scatter([0], [0])
    ax.set_box_aspect(1)
    plt.show()

else:
    Exception("Could not connect to client")