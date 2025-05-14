from breezyslam.algorithms import *
from breezyslam.sensors import Laser
from roboviz import MapVisualizer
from heiner_comunication.lidar import *
from heiner_comunication.odometry import *
import roslibpy

import signal

MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 10

lidarModel = Laser(scan_size=500, scan_rate_hz=10, detection_angle_degrees=360, distance_no_detection_mm=10000, detection_margin=0, offset_mm=0)

slam = RMHC_SLAM(lidarModel, MAP_SIZE_PIXELS, MAP_SIZE_METERS)
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

stop = False

visualizer = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS)

client = roslibpy.Ros(host='192.168.149.1', port=9091)
client.run()

pose = [0, 0, 0]

try:
    if client.is_connected:
        while not stop:

            lidar_ranges_m = get_lidar_data_once(client=client).ranges
            lidar_ranges_m = lidar_ranges_m[0:500]
            lidar_ranges_mm = [range * 1000 for range in lidar_ranges_m]
            # odom_data = get_odometry_data_once(client=client)

            slam.update(lidar_ranges_mm)

            pose[0], pose[1], pose[2] = slam.getpos()

            slam.getmap(mapbytes)

            visualizer.display(pose[0], pose[1], pose[2], mapbytes)

        client.terminate()

    else:
        Exception("Could not connect to client")

except Exception as error:
    client.terminate()
    print("Exception")
    print(error.with_traceback())


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    global stop
    stop = True


signal.signal(signal.SIGINT, signal_handler)