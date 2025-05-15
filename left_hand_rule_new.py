import roslibpy
import math
import time
from heiner_comunication.lidar import get_lidar_data_once, LidarFrame
from heiner_comunication.motor_control import move, rotate

# Parameter - Adjusted for 35cm corridors
FORWARD_SPEED = 0.7  # Reduced for better control
ROTATE_SPEED = 0.4   # Slower rotation for more precise turning
FORWARD_DURATION = 0.5  # Duration for small forward movement
TURN_DURATION = 1.0     # Duration for small turn

# Thresholds - Optimized for safer navigation
FRONT_ANGLE = 0
FRONT_SPREAD = math.radians(30)  # +-30 degrees
RIGHT_ANGLE = 3 * math.pi / 2
RIGHT_SPREAD = math.radians(30)  # +-30 degrees
DISTANCE_THRESHOLD = 0.20  # Increased for safety
RIGHT_OPEN_THRESHOLD = 0.30  # Increased for safer detection
IDEAL_WALL_DISTANCE = 0.2  # Increased - safer distance from wall
WALL_CORRECTION_FACTOR = 0.6  # Gentler correction factor

# Kreisfahrt-Erkennung
MAX_RIGHT_TURNS = 6

# Position tracking class
class RobotPosition:
    def __init__(self):
        self.x = 0  # Starting at (0,0)
        self.y = 0
        self.orientation = 0  # In radians, 0 is forward
        
    def update(self, movement, rotation):
        """Update position based on movement and rotation"""
        self.orientation += rotation
        # Normalize orientation to -pi to pi
        self.orientation = (self.orientation + math.pi) % (2 * math.pi) - math.pi
        
        # Calculate new position
        self.x += movement * math.cos(self.orientation)
        self.y += movement * math.sin(self.orientation)
        
    def get_coordinates(self):
        return (self.x, self.y)


def get_distance(lidar:LidarFrame, angle, spread):
    """Gibt den durchschnittlichen Abstand um einen bestimmten Winkel."""
    return lidar.get_value_around_angle(angle, spread)


def is_front_clear(lidar):
    """Check if front path is clear with adjusted threshold"""
    distance = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
    return distance > DISTANCE_THRESHOLD


def is_right_clear(lidar):
    """Check if right path is clear with adjusted threshold"""
    distance = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
    return distance > RIGHT_OPEN_THRESHOLD


def get_right_opening_angle(lidar: LidarFrame):
    """Bestimme, wie weit der freie Raum rechts offen ist."""
    start_angle = RIGHT_ANGLE - math.radians(90)
    stop_angle = RIGHT_ANGLE + math.radians(30)
    scan_data = lidar.get_values_between_angles(start_angle, stop_angle)

    max_distance = 0
    best_angle = RIGHT_ANGLE

    if len(scan_data) == 0:
        return -math.radians(45)  # Fallback falls keine Daten vorhanden

    for i, distance in enumerate(scan_data):
        if distance > max_distance:
            max_distance = distance
            # Berechne den aktuellen Winkel exakt
            angle = start_angle + (stop_angle - start_angle) * i / len(scan_data)
            best_angle = angle

    # Winkel relativ zur Front normalisieren (-pi bis +pi)
    angle_diff = (best_angle - FRONT_ANGLE + math.pi) % (2 * math.pi) - math.pi
    return angle_diff


# Container detection removed as requested


def adaptive_forward_movement(client, lidar):
    """Move forward at a speed proportional to the available space"""
    front_distance = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
    # Cap speed between 0.3 and 1.0 based on available distance
    adaptive_speed = min(max(front_distance * 0.8, 0.3), FORWARD_SPEED)
    move(client, adaptive_speed, 0, FORWARD_DURATION)
    return adaptive_speed * FORWARD_DURATION  # Return the actual distance moved


def maintain_wall_distance(client, lidar):
    """Adjust trajectory to maintain safe distance from right wall"""
    right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
    
    # Only correct if we're too close to the wall
    if right_dist < IDEAL_WALL_DISTANCE - 0.05:
        # Gentle correction away from wall (positive = left turn)
        correction = (IDEAL_WALL_DISTANCE - right_dist) * WALL_CORRECTION_FACTOR
        # Apply gentle correction while moving slowly
        move(client, FORWARD_SPEED * 0.5, correction, FORWARD_DURATION * 0.8)
        return True
    # Or if we're too far from wall
    elif right_dist > IDEAL_WALL_DISTANCE + 0.1:
        # Gentle correction toward wall (negative = right turn)
        correction = (IDEAL_WALL_DISTANCE - right_dist) * WALL_CORRECTION_FACTOR * 0.7
        # Apply gentler correction when moving closer to wall
        move(client, FORWARD_SPEED * 0.5, correction, FORWARD_DURATION * 0.8)
        return True
    return False


def move_forward(client):
    move(client, FORWARD_SPEED, 0, FORWARD_DURATION)
    return FORWARD_SPEED * FORWARD_DURATION


def move_forward_a_bit(client):
    move(client, FORWARD_SPEED * 0.2, 0, FORWARD_DURATION * 0.6)
    return FORWARD_SPEED * 0.2 * FORWARD_DURATION * 0.6


def move_backward(client):
    """Bewege den Roboter rückwärts."""
    move(client, -FORWARD_SPEED, 0, FORWARD_DURATION)
    return -FORWARD_SPEED * FORWARD_DURATION


def move_backward_a_bit(client):
    """Bewege den Roboter ein kleines Stück rückwärts."""
    move(client, -FORWARD_SPEED * 0.15, 0, FORWARD_DURATION * 0.4)
    return -FORWARD_SPEED * 0.15 * FORWARD_DURATION * 0.4


def rotate_right(client):
    rotate(client=client, speed=-ROTATE_SPEED, timeout=TURN_DURATION)
    return -ROTATE_SPEED * TURN_DURATION


def rotate_left(client):
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION)
    return ROTATE_SPEED * TURN_DURATION


def rotate_left_agressive(client):
    """Aggressive Linksdrehung."""
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION * 2.3)
    return ROTATE_SPEED * TURN_DURATION * 2.3


def cautious_right_turn(client, position):
    """Mehrstufige vorsichtige Rechtsdrehung mit kleinen Bewegungen."""
    for _ in range(3):
        rotation_amount = -ROTATE_SPEED * 0.4  # Amount of rotation
        rotate(client=client, speed=-ROTATE_SPEED, timeout=0.4)
        position.update(0, rotation_amount)
        
        movement_amount = FORWARD_SPEED * 0.2 * FORWARD_DURATION * 0.6
        move_forward_a_bit(client)
        position.update(movement_amount, 0)
        
        lidar = get_lidar_data_once(client, True)
        if not is_right_clear(lidar):
            break


def improved_wall_follower(client):
    """Enhanced wall follower with better efficiency for narrow corridors"""
    right_turn_counter = 0
    position = RobotPosition()
    
    while True:
        lidar = get_lidar_data_once(client, True)
        
        # First check front clearance for safety
        front_distance = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
        if front_distance < DISTANCE_THRESHOLD - 0.05:  # Dangerously close to front wall
            print("⚠️ Too close to front wall - backing up")
            move_backward_a_bit(client)
            rotate_left(client)
            continue
            
        # Check right wall status
        right_distance = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
        
        # First priority: follow right wall at safe distance if present
        if is_front_clear(lidar) and right_distance < RIGHT_OPEN_THRESHOLD and right_distance > 0.05:
            # Right wall exists and we can move forward
            if not maintain_wall_distance(client, lidar):
                # If no correction needed, move forward at appropriate speed
                speed_factor = min(max(front_distance / 0.5, 0.4), 1.0) # Scale 0.4-1.0 based on space
                move(client, FORWARD_SPEED * speed_factor, 0, FORWARD_DURATION)
                position.update(FORWARD_SPEED * speed_factor * FORWARD_DURATION, 0)
            else:
                # If correction was applied, estimate movement (simplified)
                position.update(FORWARD_SPEED * 0.5 * FORWARD_DURATION * 0.8, 0)
            right_turn_counter = 0
            
        # Second priority: turn right when right wall disappears
        elif is_right_clear(lidar) or right_distance > RIGHT_OPEN_THRESHOLD:
            right_turn_counter += 1
            
            # Calculate more conservative turning angle (smaller turn)
            right_angle = get_right_opening_angle(lidar)
            limited_angle = max(-math.radians(45), min(right_angle, -math.radians(10)))
            
            print(f"🔄 Right open, turning {math.degrees(limited_angle):.1f}°")
            
            # Execute careful turn
            turn_duration = abs(limited_angle) / (ROTATE_SPEED * 2)
            rotate(client=client, speed=-ROTATE_SPEED * 0.8, timeout=turn_duration)
            position.update(0, -limited_angle)  # Update orientation
            
            # Cautiously move forward after turning
            new_lidar = get_lidar_data_once(client, True)
            if is_front_clear(new_lidar):
                distance_moved = move_forward_a_bit(client)
                position.update(distance_moved, 0)
                
        # Third priority: move forward carefully if front is clear
        elif is_front_clear(lidar):
            # Adaptive speed based on available space
            speed_factor = min(max(front_distance / 0.5, 0.4), 1.0)
            move(client, FORWARD_SPEED * speed_factor, 0, FORWARD_DURATION)
            position.update(FORWARD_SPEED * speed_factor * FORWARD_DURATION, 0)
            right_turn_counter = 0
            
        # Final case: turn left when blocked
        else:
            right_turn_counter = 0
            # Smaller left turn for more precise movement
            rotate(client=client, speed=ROTATE_SPEED * 0.8, timeout=TURN_DURATION * 0.8)
            position.update(0, ROTATE_SPEED * 0.8 * TURN_DURATION * 0.8)
            
        # Handle potential loops more gently
        if right_turn_counter > MAX_RIGHT_TURNS:
            print("🚨 Possible loop detected - executing gentle escape maneuver")
            # More careful escape procedure
            move_backward_a_bit(client)
            position.update(-FORWARD_SPEED * 0.15 * FORWARD_DURATION * 0.4, 0)
            
            # Less aggressive left turn
            rotate(client=client, speed=ROTATE_SPEED * 0.8, timeout=TURN_DURATION * 4)
            position.update(0, ROTATE_SPEED * 0.8 * TURN_DURATION * 4)
            
            # Make sure front is clear before moving
            check_lidar = get_lidar_data_once(client, True)
            if is_front_clear(check_lidar):
                move_forward_a_bit(client)
                position.update(FORWARD_SPEED * 0.2 * FORWARD_DURATION * 0.6, 0)
            
            right_turn_counter = 0
            
        print(f"Position: ({position.x:.2f}, {position.y:.2f}), Orientation: {math.degrees(position.orientation):.1f}°")
        time.sleep(0.05)


def main(client):
    """Main function to start the robot"""
    print("🤖 Starting improved wall follower algorithm")
    print("🧭 Starting at position (0,0)")
    print("⚙️ Parameters optimized for 35cm corridors")
    improved_wall_follower(client)


# --- Einstiegspunkt ---
if __name__ == "__main__":
    client = roslibpy.Ros(host='192.168.149.1', port=9091)

    try:
        client.run()
        if client.is_connected:
            print("✅ Connected to ROSBridge")
            main(client)
        else:
            print("❌ Verbindung zu ROSBridge fehlgeschlagen")
    except KeyboardInterrupt:
        print("⛔️ Beendet durch Tasteneingabe")
        stop_cmd = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist').publish(stop_cmd)
        time.sleep(1)
        client.terminate()
    except Exception as e:
        print(f"❌ Error: {e}")
        stop_cmd = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        if client.is_connected:
            roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist').publish(stop_cmd)
            time.sleep(1)
            client.terminate()