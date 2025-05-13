import roslibpy
import time
import json

# Verbindung zu ROSBridge auf Port 9091
client = roslibpy.Ros(host='192.168.149.1', port=9091)

client.run()

def message_callback(msg):
    print(msg.keys())
    print(f"Received message: {msg['pose']['position']}")


if client.is_connected:
    print("Connected to ROSBridge!")

    # Subscribe to a topic (e.g., '/chatter')
    subscriber = roslibpy.Topic(client, '/odom_raw', 'nav_msgs/msg/Odometry')
    
    subscriber.subscribe(message_callback)

    # Keep the subscriber running
    try:
        input("Press Enter to exit...\n")
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.unsubscribe()
        client.terminate()
        print("Disconnected from ROSBridge.")
else:
    print("Failed to connect to ROSBridge.")