from heiner_comunication import topic_getter
import roslibpy
import time

class OdometryData:
    def __init__(self, data:dict):
        self.data = data
        self.position = topic_getter.Vector3(data['pose']['pose']['position']['x'],
                              data['pose']['pose']['position']['y'],
                              data['pose']['pose']['position']['z'])
        
        self.orientation = topic_getter.Vector3(data['pose']['pose']['orientation']['x'],
                                  data['pose']['pose']['orientation']['y'],
                                  data['pose']['pose']['orientation']['z'])
        
        self.covariance = data['pose']['covariance']

    def __repr__(self):
        return f"OdometryData(position={self.position}, orientation={self.orientation}, covariance={self.covariance})"
    




def get_odometry_data_once(client:roslibpy.Ros) -> OdometryData:
    return OdometryData(
        topic_getter.get_topic_once(
            client=client,
            topic_name='/odom_raw',
            message_type='nav_msgs/msg/Odometry',
            timeout=5
        )
    )


# Hier ein Dictionary, das die Null-Odometrie darstellt
ZERO_MESSAGE = {
    'pose': {
        'pose': {
            'position': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0  # Keine Drehung
            }
        },
        'covariance': [1e-09, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 1e-09, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-09]
    }
}

def reset_odometry(client: roslibpy.Ros, timeout: float = 1) -> None:
    # Topic für die Odometrie
    odom_topic = '/odom'
    
    # Publisher für das Odometrie-Topic erstellen
    talker = roslibpy.Topic(client, odom_topic, 'nav_msgs/msg/Odometry')

    # Warten, bis das Topic erfolgreich advertised wurde
    talker.advertise()
    
    # Nachricht veröffentlichen, die die Odometrie auf Null setzt
    talker.publish({
        'header': {
            'stamp': {'secs': 0, 'nsecs': 0},
            'frame_id': 'odom'
        },
        'pose': ZERO_MESSAGE['pose']
    })
    print("Odometrie wurde auf Null gesetzt.")
    
    # Warten, damit die Nachricht durch ROS verarbeitet werden kann
    time.sleep(timeout)
    
    # Publisher unadvertise, um Ressourcen freizugeben
    talker.unadvertise()
