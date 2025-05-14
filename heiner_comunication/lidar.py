import roslibpy
from heiner_comunication import topic_getter
import math

class LidarFrame:

    def __init__(self, data: dict):
        self.data = data
        self.angle_min = data['angle_min']
        self.angle_max = data['angle_max']
        self.angle_increment = data['angle_increment']
        self.ranges = data['ranges']
        # Replace None values with the previous value in the list
        for i in range(1, len(self.ranges)):
            if self.ranges[i] is None:
                self.ranges[i] = self.ranges[i - 1]

        if None in self.ranges:
            print("Uh Oh :(")
        

    def get_avgr_value_between(self, angle_min: float, angle_max: float) -> list:
        """
        Gibt die Werte zwischen angle_min und angle_max zurück.
        
        :param angle_min: Minimaler Winkel
        :param angle_max: Maximaler Winkel
        :return: Liste der Werte zwischen angle_min und angle_max
        """
        angle_min = angle_min % (2 * math.pi)
        angle_max = angle_max % (2 * math.pi)

        if angle_min > angle_max:
            # Handle wrapping around 0
            ranges_part1 = self.ranges[int((angle_min - self.angle_min) / self.angle_increment):]
            ranges_part2 = self.ranges[:int((angle_max - self.angle_min) / self.angle_increment)]
            selected_ranges = ranges_part1 + ranges_part2
        else:
            start_index = int((angle_min - self.angle_min) / self.angle_increment)
            end_index = int((angle_max - self.angle_min) / self.angle_increment)
            # Ensure indices are within bounds
            start_index = max(0, min(start_index, len(self.ranges) - 1))
            end_index = max(0, min(end_index, len(self.ranges) - 1))
            selected_ranges = self.ranges[start_index:end_index]

        return sum(selected_ranges) / len(selected_ranges) if selected_ranges else 0
    
    def get_value_around_angle(self, angle: float, radius:float=math.pi * 2 / 4) -> float:
        """
        Gibt den Wert um einen bestimmten Winkel zurück.
        
        :param angle: Winkel
        :param radius: Radius um den Winkel
        :return: Wert um den Winkel
        """
        start_angle = angle - radius / 2
        end_angle = angle + radius / 2
        if end_angle > 2 * math.pi:
            end_angle -= 2 * math.pi
        if end_angle < 0:
            end_angle += 2 * math.pi
        return self.get_avgr_value_between(start_angle, end_angle)

    



    def __repr__(self):
        return f"LidarFrame(angle_min={self.angle_min}, angle_max={self.angle_max}, angle_increment={self.angle_increment}, ranges={len(self.ranges)})"

def get_lidar_data_once(client:roslibpy.Ros) -> LidarFrame:
    """
    LiDAR-Daten einmal abfragen.
    
    :param client: ROS-Client
    :return: LiDAR-Daten
    """
    return LidarFrame(topic_getter.get_topic_once(
        client=client,
        topic_name='/scan_raw',
        message_type='sensor_msgs/msg/LaserScan',
        timeout=5
    ))