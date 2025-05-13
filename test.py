import heiner_comunication.topic_getter as tg
import roslibpy
import time


# Verbindung zu ROSBridge auf Port 9091
client = roslibpy.Ros(host='192.168.149.1', port=9091)

client.run()


if client.is_connected:
    msg = tg.get_topic_once(client, '/odom_raw', 'nav_msgs/msg/Odometry')
    print(msg)


    client.terminate()