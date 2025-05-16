import roslibpy
import math
import time
import numpy as np
import os
import random
from heiner_comunication.lidar import get_lidar_data_once, LidarFrame
from heiner_comunication.motor_control import move, rotate

# Parameter settings
FORWARD_SPEED = 0.7
ROTATE_SPEED = 0.4
FORWARD_DURATION = 0.5
TURN_DURATION = 1.0

# Distance thresholds
FRONT_ANGLE = 0
FRONT_SPREAD = math.radians(30)
RIGHT_ANGLE = 3 * math.pi / 2
RIGHT_SPREAD = math.radians(30)
LEFT_ANGLE = math.pi / 2
LEFT_SPREAD = math.radians(30)
BACK_ANGLE = math.pi
BACK_SPREAD = math.radians(30)
DISTANCE_THRESHOLD = 0.20  # Minimum safe distance
WALL_FOLLOW_THRESHOLD = 0.30  # Distance to follow wall

# Reinforcement Learning parameters
LEARNING_RATE = 0.2
DISCOUNT_FACTOR = 0.8
EXPLORATION_RATE = 0.2
MAX_STEPS = 2000
Q_TABLE_FILE = "q_table.npy"

# State space parameters
DIST_CLOSE = 0.18
DIST_MEDIUM = 0.35

# Custom quaternion to yaw conversion
def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw (in radians)."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

# Odometry Handler Class
class OdometryHandler:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update = time.time()
        self.odom_sub = None
        self.data_valid = False
        self.max_stale_time = 1.0

    def odom_callback(self, message):
        """Callback to process odometry data."""
        try:
            self.x = message['pose']['pose']['position']['x']
            self.y = message['pose']['pose']['position']['y']
            orientation_q = message['pose']['pose']['orientation']
            self.yaw = quaternion_to_yaw(
                orientation_q['x'],
                orientation_q['y'],
                orientation_q['z'],
                orientation_q['w']
            )
            self.last_update = time.time()
            self.data_valid = True
        except KeyError as e:
            print(f"❌ Odometry message missing field: {e}")
            self.data_valid = False

    def is_data_fresh(self):
        """Check if odometry data is fresh."""
        return self.data_valid and (time.time() - self.last_update) < self.max_stale_time

    def subscribe(self, client):
        """Subscribe to /odom topic."""
        self.odom_sub = roslibpy.Topic(client, '/odom', 'nav_msgs/Odometry')
        self.odom_sub.subscribe(self.odom_callback)
        print("📍 Subscribed to /odom topic")

    def unsubscribe(self):
        """Unsubscribe from /odom topic."""
        if self.odom_sub:
            self.odom_sub.unsubscribe()

# Robot Position Class using Odometry
class RobotPosition:
    def __init__(self, client):
        self.odom = OdometryHandler()
        self.odom.subscribe(client)
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.visited_positions = []
        self.position_threshold = 0.1
        self.max_positions = 100  # Keep only the last 100 positions

    def update(self):
        """Update position from odometry data."""
        if not self.odom.is_data_fresh():
            print("⚠️ Odometry data stale!")
            return False
        self.x = self.odom.x
        self.y = self.odom.y
        self.orientation = self.odom.yaw
        self.orientation = (self.orientation + math.pi) % (2 * math.pi) - math.pi
        self.visited_positions.append((self.x, self.y))
        if len(self.visited_positions) > self.max_positions:
            self.visited_positions = self.visited_positions[-self.max_positions:]
        return True

    def check_loop(self):
        """Check if robot is revisiting a position."""
        if len(self.visited_positions) < 20:  # Need enough history
            return False
        current_pos = (self.x, self.y)
        for pos in self.visited_positions[:-10]:  # Skip the most recent positions
            dist = math.sqrt((current_pos[0] - pos[0])**2 + (current_pos[1] - pos[1])**2)
            if dist < self.position_threshold:
                return True
        return False
        
    def get_distance_moved(self, prev_x, prev_y):
        """Calculate distance moved from previous position."""
        return math.sqrt((self.x - prev_x)**2 + (self.y - prev_y)**2)

# Basic movement functions
def get_distance(lidar: LidarFrame, angle, spread):
    """Get valid distance from LiDAR at given angle"""
    distance = lidar.get_value_around_angle(angle, spread)
    if distance is None or np.isnan(distance) or distance < 0:
        return float('inf')  # Return infinite for invalid readings
    return distance

def move_forward(client):
    """Move robot forward"""
    move(client, FORWARD_SPEED, 0, FORWARD_DURATION)

def move_backward(client):
    """Move robot backward (for emergency)"""
    move(client, -FORWARD_SPEED * 0.15, 0, FORWARD_DURATION * 0.4)

def rotate_right(client):
    """Rotate robot right"""
    rotate(client=client, speed=-ROTATE_SPEED, timeout=TURN_DURATION)

def rotate_left(client):
    """Rotate robot left"""
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION)

def execute_action(client, action):
    """Execute specified action"""
    if action == 0:  # Forward
        move_forward(client)
    elif action == 1:  # Right turn
        rotate_right(client)
    elif action == 2:  # Left turn
        rotate_left(client)
    elif action == 3:  # Backward
        move_backward(client)
    else:
        print(f"❌ Invalid action: {action}")

# Simple RL Agent
class RLAgent:
    def __init__(self):
        # State space: 3 bins for each direction (front, right, left, back) = 3^4 = 81 states
        self.num_states = 81
        self.num_actions = 4  # forward, right, left, backward
        self.load_or_create_q_table()

    def load_or_create_q_table(self):
        """Load existing Q-table or create a new one"""
        if os.path.exists(Q_TABLE_FILE):
            try:
                self.q_table = np.load(Q_TABLE_FILE)
                if self.q_table.shape == (self.num_states, self.num_actions):
                    print(f"✅ Loaded Q-table with {np.count_nonzero(self.q_table)} non-zero values")
                else:
                    print("⚠️ Invalid Q-table shape, creating new")
                    self.q_table = np.zeros((self.num_states, self.num_actions))
            except Exception as e:
                print(f"❌ Error loading Q-table: {e}")
                self.q_table = np.zeros((self.num_states, self.num_actions))
        else:
            print("Creating new Q-table")
            self.q_table = np.zeros((self.num_states, self.num_actions))

    def get_state(self, lidar):
        """Convert LiDAR readings to state index"""
        front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
        right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
        left_dist = get_distance(lidar, LEFT_ANGLE, LEFT_SPREAD)
        back_dist = get_distance(lidar, BACK_ANGLE, BACK_SPREAD)
        
        # Discretize distances into 3 bins each (0:close, 1:medium, 2:far)
        front_bin = 0 if front_dist < DIST_CLOSE else (1 if front_dist < DIST_MEDIUM else 2)
        right_bin = 0 if right_dist < DIST_CLOSE else (1 if right_dist < DIST_MEDIUM else 2)
        left_bin = 0 if left_dist < DIST_CLOSE else (1 if left_dist < DIST_MEDIUM else 2)
        back_bin = 0 if back_dist < DIST_CLOSE else (1 if back_dist < DIST_MEDIUM else 2)
        
        # Combine bins into a single state index
        state = front_bin * 27 + right_bin * 9 + left_bin * 3 + back_bin
        return state, (front_dist, right_dist, left_dist, back_dist)

    def get_action(self, state):
        """Choose action using epsilon-greedy policy"""
        if random.random() < EXPLORATION_RATE:
            return random.randint(0, self.num_actions - 1)  # Explore
        else:
            return np.argmax(self.q_table[state])  # Exploit

    def update_q_value(self, state, action, reward, next_state):
        """Update Q-value using Q-learning formula"""
        current_q = self.q_table[state, action]
        max_next_q = np.max(self.q_table[next_state])
        new_q = current_q + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max_next_q - current_q)
        self.q_table[state, action] = new_q

    def save_q_table(self):
        """Save Q-table to file"""
        try:
            q_table_copy = np.copy(self.q_table)  # Create a clean copy
            np.save(Q_TABLE_FILE, q_table_copy)
            print("💾 Q-table saved")
        except Exception as e:
            print(f"❌ Error saving Q-table: {e}")
            try:
                np.savetxt(f"{Q_TABLE_FILE}.txt", self.q_table)
                print("💾 Q-table saved as text backup")
            except Exception as e2:
                print(f"❌ Error saving backup Q-table: {e2}")

def calculate_reward(distances, prev_distances, action, position, prev_x, prev_y):
    """Calculate reward based on the robot's situation and movement"""
    front_dist, right_dist, left_dist, back_dist = distances
    reward = 0
    
    # Safety penalties for being too close to obstacles
    if front_dist < 0.12:
        reward -= 8  # Heavy penalty for nearly hitting front obstacle
    if right_dist < 0.12:
        reward -= 3  # Penalty for being too close to right wall
    if left_dist < 0.12:
        reward -= 3  # Penalty for being too close to left wall
        
    # Reward for forward progress in open space
    if front_dist > 0.35 and action == 0:
        reward += 4  # Reward for moving forward in open space
        
    # Reward for wall-following behavior
    if 0.15 < right_dist < 0.25 and front_dist > 0.3:
        reward += 2  # Reward for maintaining ideal wall distance
    
    # Reward for effective turns
    if action == 1 and prev_distances and prev_distances[0] < 0.2 and front_dist > 0.3:
        reward += 3  # Reward for turning away from obstacle
    if action == 2 and prev_distances and prev_distances[0] < 0.2 and front_dist > 0.3:
        reward += 3  # Reward for turning away from obstacle
    
    # Reward based on position change (from odometry)
    dist_moved = position.get_distance_moved(prev_x, prev_y)
    if dist_moved > 0.01:  # Only reward significant movements
        reward += dist_moved * 5
    
    # Penalize loops
    if position.check_loop():
        reward -= 10
        print("🔄 Loop detected! Penalizing...")
        
    return reward

def stop_robot(client):
    """Emergency stop function - ensures robot completely stops"""
    print("🛑 Emergency stopping robot...")
    stop_cmd = {
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }
    # Create topic and send stop command multiple times
    stop_topic = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
    for _ in range(3):  # Send multiple times to ensure it's received
        stop_topic.publish(stop_cmd)
        time.sleep(0.1)  # Small delay between commands
    print("✋ Robot stopped")

def run_robot(client, max_steps=MAX_STEPS):
    """Main function to run the RL-based robot navigation with odometry"""
    agent = RLAgent()
    position = RobotPosition(client)
    steps = 0
    total_reward = 0
    prev_distances = None
    states_visited = set()
    
    print("🤖 Starting simple RL robot navigation with odometry")
    
    try:
        # Wait for initial odometry data
        waiting_time = 0
        while not position.update() and waiting_time < 5:
            print("⏳ Waiting for initial odometry data...")
            time.sleep(0.5)
            waiting_time += 0.5
        
        if waiting_time >= 5:
            print("❌ Failed to get initial odometry data")
            return
            
        while steps < max_steps:
            # Store previous position
            prev_x, prev_y = position.x, position.y
            
            # Get LiDAR data
            lidar = get_lidar_data_once(client, True)
            
            # Emergency backup if too close to front wall
            front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
            if front_dist < 0.12:
                print("⚠️ Emergency backup - too close to wall")
                move_backward(client)
                time.sleep(0.2)
                position.update()
                lidar = get_lidar_data_once(client, True)
            
            # Get current state
            state, distances = agent.get_state(lidar)
            states_visited.add(state)
            
            # Choose and execute action
            action = agent.get_action(state)
            execute_action(client, action)
            time.sleep(0.1)  # Give time for action to complete
            
            # Update position
            if not position.update():
                print("⚠️ Failed to update position, using last known position")
            
            # Get new state and calculate reward
            new_lidar = get_lidar_data_once(client, True)
            new_state, new_distances = agent.get_state(new_lidar)
            
            reward = calculate_reward(new_distances, prev_distances, action, position, prev_x, prev_y)
            total_reward += reward
            
            # Update Q-table
            agent.update_q_value(state, action, reward, new_state)
            
            # Store current data for next iteration
            prev_distances = new_distances
            
            # Log progress
            if steps % 10 == 0:
                action_names = ["Forward", "Right", "Left", "Back"]
                action_name = action_names[action] if 0 <= action < len(action_names) else f"Unknown({action})"
                print(f"Step {steps}: Action={action_name}, Reward={reward:.1f}, Pos=({position.x:.2f}, {position.y:.2f})")
                
            # Periodically save Q-table
            if steps % 100 == 0:
                coverage = len(states_visited) / agent.num_states * 100
                print(f"🔍 State coverage: {len(states_visited)}/{agent.num_states} states ({coverage:.1f}%)")
                agent.save_q_table()
            
            steps += 1
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("⛔ Stopped by user")
        stop_robot(client)  # Call emergency stop
    except Exception as e:
        print(f"❌ Error: {e}")
        stop_robot(client)  # Call emergency stop
        import traceback
        traceback.print_exc()
    finally:
        # Always save Q-table before exiting
        agent.save_q_table()
        # Clean up odometry subscription
        position.odom.unsubscribe()
    
    # Final report
    coverage = len(states_visited) / agent.num_states * 100
    print(f"🏁 Run completed: {steps} steps, Total reward: {total_reward:.1f}")
    print(f"🔍 State coverage: {len(states_visited)}/{agent.num_states} states ({coverage:.1f}%)")

# Entry point
if __name__ == "__main__":
    client = roslibpy.Ros(host='192.168.149.1', port=9091)
    try:
        client.run()
        if client.is_connected:
            print("✅ Connected to ROSBridge")
            run_robot(client)
        else:
            print("❌ Connection to ROSBridge failed")
    except KeyboardInterrupt:
        print("⛔️ Terminated by keyboard input")
        stop_robot(client)  # Call emergency stop
    except Exception as e:
        print(f"❌ Error: {e}")
        if client.is_connected:
            stop_robot(client)  # Call emergency stop
    finally:
        if client.is_connected:
            client.terminate()
            print("🔌 ROS connection terminated")