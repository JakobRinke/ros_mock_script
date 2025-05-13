import roslibpy
import time


class Vector3:
    def __init__(self, x:float, y:float, z:float):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"Vector3(x={self.x}, y={self.y}, z={self.z})"


def get_topic_once(client:roslibpy.Ros, topic_name, message_type, timeout=5):
    result = {'message': None}

    def callback(message):
        result['message'] = message
        listener.unsubscribe()

    listener = roslibpy.Topic(client, topic_name, message_type)
    listener.subscribe(callback)

    start_time = time.time()
    while result['message'] is None:
        if time.time() - start_time > timeout:
            listener.unsubscribe()
            raise TimeoutError(f'Keine Nachricht von {topic_name} innerhalb von {timeout} Sekunden empfangen.')
        time.sleep(0.1)

    return result['message']