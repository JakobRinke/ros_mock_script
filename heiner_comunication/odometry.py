from heiner_comunication import topic_getter
import roslibpy


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