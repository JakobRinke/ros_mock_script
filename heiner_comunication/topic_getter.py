import roslibpy
import time

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