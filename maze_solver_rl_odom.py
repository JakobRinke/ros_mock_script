import roslibpy
import math
import time
import numpy as np
import os
import random
from heiner_comunication.lidar import get_lidar_data_once, LidarFrame
from heiner_comunication.motor_control import move, rotate

# Parameter settings - optimized for 35cm corridors
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
        self.max_positions = 1000

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
        current_pos = (self.x, self.y)
        for pos in self.visited_positions[:-10]:
            dist = math.sqrt((current_pos[0] - pos[0])**2 + (current_pos[1] - pos[1])**2)
            if dist < self.position_threshold:
                return True
        return False

# Custom Position object for copying
class PositionData:
    def __init__(self, x=0.0, y=0.0, orientation=0.0):
        self.x = x
        self.y = y
        self.orientation = orientation

# Enhanced RL Agent with Orientation
class EnhancedRLAgent:
    def __init__(self):
        self.num_states = 81 * 4  # 3^4 * 4 orientation buckets
        self.num_actions = 4
        self.init_q_table()

    def state_index(self, front, right, left, back, orientation):
        orientation_bin = self.discretize_orientation(orientation)
        return (front * 27 * 4 + right * 9 * 4 + left * 3 * 4 + back * 4 + orientation_bin)

    def discretize_orientation(self, yaw):
        angle = (yaw + math.pi) % (2 * math.pi) - math.pi
        if -math.pi/4 <= angle < math.pi/4:
            return 0
        elif math.pi/4 <= angle < 3*math.pi/4:
            return 1
        elif 3*math.pi/4 <= angle or angle < -3*math.pi/4:
            return 2
        else:
            return 3

    def get_state(self, lidar, orientation):
        front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
        right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
        left_dist = get_distance(lidar, LEFT_ANGLE, LEFT_SPREAD)
        back_dist = get_distance(lidar, BACK_ANGLE, BACK_SPREAD)
        front_state = 0 if front_dist < DIST_CLOSE else 1 if front_dist < DIST_MEDIUM else 2
        right_state = 0 if right_dist < DIST_CLOSE else 1 if right_dist < DIST_MEDIUM else 2
        left_state = 0 if left_dist < DIST_CLOSE else 1 if left_dist < DIST_MEDIUM else 2
        back_state = 0 if back_dist < DIST_CLOSE else 1 if back_dist < DIST_MEDIUM else 2
        return self.state_index(front_state, right_state, left_state, back_state, orientation)

    def init_q_table(self):
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

    def get_action(self, state):
        if np.random.random() < EXPLORATION_RATE:
            return np.random.randint(0, self.num_actions)
        return np.argmax(self.q_table[state])

    def update_q_value(self, state, action, reward, next_state):
        current_q = self.q_table[state, action]
        max_next_q = np.max(self.q_table[next_state])
        new_q = current_q + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max_next_q - current_q)
        self.q_table[state, action] = new_q

    def save_q_table(self):
        try:
            np.save(Q_TABLE_FILE, self.q_table)
            print("💾 Q-table saved")
        except Exception as e:
            print(f"❌ Error saving Q-table: {e}")

# Basic movement functions
def get_distance(lidar: LidarFrame, angle, spread):
    distance = lidar.get_value_around_angle(angle, spread)
    if distance is None or np.isnan(distance) or distance < 0:
        print(f"⚠️ Invalid LiDAR distance at angle {angle:.2f}")
        return float('inf')
    return distance

def move_forward(client):
    move(client, FORWARD_SPEED, 0, FORWARD_DURATION)
    return FORWARD_SPEED * FORWARD_DURATION

def move_backward_a_bit(client):
    move(client, -FORWARD_SPEED * 0.15, 0, FORWARD_DURATION * 0.4)
    return -FORWARD_SPEED * 0.15 * FORWARD_DURATION * 0.4

def rotate_right(client):
    rotate(client=client, speed=-ROTATE_SPEED, timeout=TURN_DURATION)
    return -ROTATE_SPEED * TURN_DURATION

def rotate_left(client):
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION)
    return ROTATE_SPEED * TURN_DURATION

def execute_action(client, action):
    if action == 0:
        return move_forward(client)
    elif action == 1:
        return rotate_right(client)
    elif action == 2:
        return rotate_left(client)
    elif action == 3:
        return move_backward_a_bit(client)
    else:
        print(f"❌ Invalid action: {action}")
        return 0

def calculate_reward(lidar, prev_lidar, prev_action, position, prev_position):
    front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
    right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
    left_dist = get_distance(lidar, LEFT_ANGLE, LEFT_SPREAD)
    back_dist = get_distance(lidar, BACK_ANGLE, BACK_SPREAD)
    
    reward = 0
    
    # Safety penalties for being too close
    if front_dist < 0.12:
        reward -= 8
    if right_dist < 0.12:
        reward -= 5
    if left_dist < 0.12:
        reward -= 3
    if back_dist < 0.12:
        reward -= 4
    
    # Reward for open space
    if front_dist > 0.35 and prev_action == 0:
        reward += 5
    
    # Reward for movement
    if prev_position is not None:
        dist_moved = math.sqrt((position.x - prev_position.x)**2 + (position.y - prev_position.y)**2)
        if dist_moved > 0.01:
            reward += dist_moved * 5
        
        # Loop detection penalty
        if position.check_loop():
            reward -= 10
            print("🔄 Loop detected! Penalizing...")

    return reward

def rl_navigation(client, max_steps=MAX_STEPS):
    agent = EnhancedRLAgent()
    position = RobotPosition(client)
    steps = 0
    prev_lidar = None
    prev_action = None
    prev_position = None
    total_reward = 0
    states_visited = set()

    print("🤖 Starting RL-based maze navigation")
    
    try:
        while steps < max_steps:
            if not position.update():
                print("⏳ Waiting for fresh odometry data...")
                time.sleep(0.2)
                continue

            prev_position = PositionData(position.x, position.y, position.orientation)
            lidar = get_lidar_data_once(client, True)
            
            try:
                front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
                if front_dist < 0.12:
                    print("⚠️ Emergency backup - too close to wall")
                    move_backward_a_bit(client)
                    position.update()
                    lidar = get_lidar_data_once(client, True)
            except Exception as e:
                print(f"❌ LiDAR data error: {e}")
                time.sleep(0.2)
                continue

            state = agent.get_state(lidar, position.orientation)
            states_visited.add(state)
            action = agent.get_action(state)
            execute_action(client, action)
            position.update()

            new_lidar = get_lidar_data_once(client, True)
            new_state = agent.get_state(new_lidar, position.orientation)
            reward = calculate_reward(new_lidar, prev_lidar, action, position, prev_position)
            total_reward += reward

            agent.update_q_value(state, action, reward, new_state)
            
            prev_lidar = new_lidar
            prev_action = action
            prev_position = PositionData(position.x, position.y, position.orientation)

            if steps % 100 == 0:
                coverage = len(states_visited) / agent.num_states * 100
                print(f"Step {steps}: Reward: {reward:.1f}, Total Reward: {total_reward:.1f}, "
                      f"State coverage: {len(states_visited)}/{agent.num_states} ({coverage:.1f}%)")
                agent.save_q_table()

            steps += 1
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("⛔ Stopped by user")
    finally:
        agent.save_q_table()
        position.odom.unsubscribe()

    coverage = len(states_visited) / agent.num_states * 100
    print(f"🏁 Run completed: {steps} steps, Total reward: {total_reward:.1f}, "
          f"State coverage: {len(states_visited)}/{agent.num_states} ({coverage:.1f}%)")

def main(client):
    print("🤖 RL Maze-solving Robot")
    rl_navigation(client, max_steps=MAX_STEPS)

if __name__ == "__main__":
    client = roslibpy.Ros(host='192.168.149.1', port=9091)
    try:
        client.run()
        if client.is_connected:
            print("✅ Connected to ROSBridge")
            main(client)
        else:
            print("❌ Connection to ROSBridge failed")
    except KeyboardInterrupt:
        print("⛔️ Terminated by keyboard input")
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