import roslibpy
import math
import time
import numpy as np
import os
import random
import copy
from heiner_comunication.lidar import get_lidar_data_once, LidarFrame
from heiner_comunication.motor_control import move, rotate
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Parameter settings - optimized for 35cm corridors
FORWARD_SPEED = 0.7  # Speed for forward movement
ROTATE_SPEED = 0.4   # Speed for rotation
FORWARD_DURATION = 0.5  # Duration for forward movement
TURN_DURATION = 1.0     # Duration for rotation

# Distance thresholds
FRONT_ANGLE = 0
FRONT_SPREAD = math.radians(30)  # +-30 degrees
RIGHT_ANGLE = 3 * math.pi / 2
RIGHT_SPREAD = math.radians(30)  # +-30 degrees
LEFT_ANGLE = math.pi / 2  # Adding left angle (90 degrees)
BACK_ANGLE = math.pi  # Adding back angle (180 degrees)
DISTANCE_THRESHOLD = 0.20  # Front clearance threshold
RIGHT_OPEN_THRESHOLD = 0.30  # Right opening threshold
IDEAL_WALL_DISTANCE = 0.2  # Ideal distance from right wall
WALL_CORRECTION_FACTOR = 0.6  # Correction factor
BACK_SAFE_DISTANCE = 0.18  # Safe distance from rear obstacles

# Reinforcement Learning parameters - optimized for shorter runs
LEARNING_RATE = 0.2     # Increased to learn faster (was 0.1)
DISCOUNT_FACTOR = 0.8   # Slightly reduced to prioritize immediate rewards
EXPLORATION_RATE = 0.2  # Increased exploration for better coverage in shorter runs
UPDATE_INTERVAL = 1     # Update every step for faster learning
STEPS_BEFORE_RL = 200   # Reduced from 500 to start using RL sooner
Q_CONFIDENCE_THRESHOLD = 0.7  # Reduced threshold for using learned actions (was 1.0)
MAX_STEPS = 2000        # Reasonable maximum for a single run
Q_TABLE_FILE = "q_table.npy"
OLD_Q_TABLE_FILE = "q_table_old.npy"  # For backup before upgrading


# Loop detection
MAX_RIGHT_TURNS = 3

# State space parameters - adjusted for STL-19P TOF LiDAR
DIST_CLOSE = 0.18       # Distance considered "too close" - increased for TOF sensor accuracy
DIST_MEDIUM = 0.35      # Distance threshold between "medium" and "far" - adjusted for TOF range

# Odometry Handler Class
class OdometryHandler:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update = time.time()
        self.odom_sub = None

    def odom_callback(self, message):
        """Callback to process odometry data"""
        self.x = message['pose']['pose']['position']['x']
        self.y = message['pose']['pose']['position']['y']
        orientation_q = message['pose']['pose']['orientation']
        orientation_list = [orientation_q['x'], orientation_q['y'], orientation_q['z'], orientation_q['w']]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw
        self.last_update = time.time()

    def subscribe(self, client):
        """Subscribe to /odom topic"""
        self.odom_sub = roslibpy.Topic(client, '/odom', 'nav_msgs/Odometry')
        self.odom_sub.subscribe(self.odom_callback)
        print("📍 Subscribed to /odom topic")

    def unsubscribe(self):
        """Unsubscribe from /odom topic"""
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
        self.visited_positions = []  # Track positions for loop detection
        self.position_threshold = 0.1  # Threshold for considering positions "same" (meters)

    def update(self):
        """Update position from odometry data"""
        self.x = self.odom.x
        self.y = self.odom.y
        self.orientation = self.odom.yaw
        self.orientation = (self.orientation + math.pi) % (2 * math.pi) - math.pi
        self.visited_positions.append((self.x, self.y))

    def check_loop(self):
        """Check if robot is revisiting a position (loop detection)"""
        current_pos = (self.x, self.y)
        for pos in self.visited_positions[:-10]:  # Ignore recent positions
            dist = math.sqrt((current_pos[0] - pos[0])**2 + (current_pos[1] - pos[1])**2)
            if dist < self.position_threshold:
                return True
        return False

# Enhanced RL Agent with Orientation
class EnhancedRLAgent:
    def __init__(self):
        self.num_states = 81 * 4  # 81 LIDAR states * 4 orientation bins
        self.num_actions = 4
        self.init_q_table()
        self.update_counter = 0

    def state_index(self, front, right, left, back, orientation):
        """Convert state values to a single index"""
        orientation_bin = self.discretize_orientation(orientation)
        return (front * 27 * 4 + right * 9 * 4 + left * 3 * 4 + back * 4 + orientation_bin)

    def discretize_orientation(self, yaw):
        """Discretize orientation into 4 bins (0: ~North, 1: ~East, 2: ~South, 3: ~West)"""
        angle = (yaw + math.pi) % (2 * math.pi) - math.pi
        if -math.pi/4 <= angle < math.pi/4:
            return 0  # North
        elif math.pi/4 <= angle < 3*math.pi/4:
            return 1  # East
        elif 3*math.pi/4 <= angle or angle < -3*math.pi/4:
            return 2  # South
        else:
            return 3  # West

    def get_state(self, lidar, orientation):
        """Convert LIDAR data and orientation to state representation"""
        front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
        right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
        left_dist = get_distance(lidar, LEFT_ANGLE, RIGHT_SPREAD)
        back_dist = get_distance(lidar, BACK_ANGLE, FRONT_SPREAD)

        if random.random() < 0.01:
            print(f"📏 Raw distances - F:{front_dist:.2f}, R:{right_dist:.2f}, L:{left_dist:.2f}, B:{back_dist:.2f}, Yaw:{orientation:.2f}")

        front_state = 0 if front_dist < DIST_CLOSE else 1 if front_dist < DIST_MEDIUM else 2
        right_state = 0 if right_dist < DIST_CLOSE else 1 if right_dist < DIST_MEDIUM else 2
        left_state = 0 if left_dist < DIST_CLOSE else 1 if left_dist < DIST_MEDIUM else 2
        back_state = 0 if back_dist < DIST_CLOSE else 1 if back_dist < DIST_MEDIUM else 2
        return self.state_index(front_state, right_state, left_state, back_state, orientation)

    def init_q_table(self):
        """Initialize or load Q-table, handle transition from old format"""
        print(f"📁 Q-table file location: {os.path.abspath(Q_TABLE_FILE)}")
        if os.path.exists(Q_TABLE_FILE):
            try:
                existing_table = np.load(Q_TABLE_FILE)
                if existing_table.shape == (81, 4):  # Old format (no orientation)
                    print("🔄 Upgrading Q-table to include orientation")
                    np.save(OLD_Q_TABLE_FILE, existing_table)
                    self.q_table = np.zeros((self.num_states, self.num_actions))
                    for state in range(81):
                        for action in range(4):
                            for orient in range(4):
                                new_state = state * 4 + orient
                                self.q_table[new_state, action] = existing_table[state, action] * 0.8
                    print("🧠 Knowledge transferred to new state representation")
                elif existing_table.shape == (self.num_states, self.num_actions):
                    self.q_table = existing_table
                    print(f"✅ Loaded existing Q-table with {np.count_nonzero(self.q_table)} non-zero values")
                    return
                else:
                    print(f"⚠️ Unexpected Q-table shape: {existing_table.shape}")
                    self.q_table = np.zeros((self.num_states, self.num_actions))
            except Exception as e:
                print(f"❌ Error inspecting Q-table: {e}")
                self.q_table = np.zeros((self.num_states, self.num_actions))
        else:
            print("Creating new enhanced Q-table")
            self.q_table = np.zeros((self.num_states, self.num_actions))

    def get_action(self, state):
        """Select action using epsilon-greedy policy"""
        if np.random.random() < EXPLORATION_RATE:
            return np.random.randint(0, self.num_actions)
        else:
            return np.argmax(self.q_table[state])

    def update_q_value(self, state, action, reward, next_state):
        """Update Q-value"""
        self.update_counter += 1
        current_q = self.q_table[state, action]
        max_next_q = np.max(self.q_table[next_state])
        new_q = current_q + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max_next_q - current_q)
        self.q_table[state, action] = new_q
        if self.update_counter % 100 == 0:
            self.save_q_table()

    def save_q_table(self):
        """Save Q-table to file"""
        try:
            np.save(Q_TABLE_FILE, self.q_table)
            print("💾 Q-table saved")
        except Exception as e:
            print(f"❌ Error saving Q-table: {e}")

# Basic movement functions
def get_distance(lidar: LidarFrame, angle, spread):
    """Get average distance at specified angle"""
    return lidar.get_value_around_angle(angle, spread)

def is_front_clear(lidar):
    """Check if front is clear"""
    return get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD) > DISTANCE_THRESHOLD

def is_right_clear(lidar):
    """Check if right side is clear"""
    return get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD) > RIGHT_OPEN_THRESHOLD

def move_forward(client):
    """Move forward at normal speed"""
    move(client, FORWARD_SPEED, 0, FORWARD_DURATION)
    return FORWARD_SPEED * FORWARD_DURATION

def move_backward_a_bit(client):
    """Move backward slightly"""
    move(client, -FORWARD_SPEED * 0.15, 0, FORWARD_DURATION * 0.4)
    return -FORWARD_SPEED * 0.15 * FORWARD_DURATION * 0.4

def rotate_right(client):
    """Rotate right"""
    rotate(client=client, speed=-ROTATE_SPEED, timeout=TURN_DURATION)
    return -ROTATE_SPEED * TURN_DURATION

def rotate_left(client):
    """Rotate left"""
    rotate(client=client, speed=ROTATE_SPEED, timeout=TURN_DURATION)
    return ROTATE_SPEED * TURN_DURATION

def execute_action(client, action):
    """Execute action based on action index"""
    if action == 0:  # Move forward
        return move_forward(client)
    elif action == 1:  # Turn right
        return rotate_right(client)
    elif action == 2:  # Turn left
        return rotate_left(client)
    elif action == 3:  # Move backward
        return move_backward_a_bit(client)
    else:
        print(f"❌ Invalid action: {action}")
        return 0

def calculate_reward(lidar, prev_lidar, prev_action, position, prev_position):
    """Calculate reward with odometry and back distance enhancements"""
    front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
    right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
    left_dist = get_distance(lidar, LEFT_ANGLE, RIGHT_SPREAD)
    back_dist = get_distance(lidar, BACK_ANGLE, FRONT_SPREAD)
    
    reward = 0
    
    # LIDAR-based rewards/penalties
    if front_dist < 0.12:
        reward -= 8
    if right_dist < 0.12:
        reward -= 5
    if left_dist < 0.12:
        reward -= 3
    if back_dist < 0.12:  # NEW: Penalize being too close to rear obstacle
        reward -= 4
    if 0.18 < right_dist < 0.28:
        reward += 3
    if front_dist > 0.35 and prev_action == 0:
        space_reward = 3
        if front_dist > 0.5:
            space_reward += 2
        if front_dist > 0.7:
            space_reward += 2
        reward += space_reward
    if right_dist < RIGHT_OPEN_THRESHOLD and right_dist > 0.18 and front_dist > 0.35:
        reward += 2
    if prev_action == 1 and right_dist > 0.4:
        reward += 3
    if prev_action == 2 and front_dist > 0.35 and prev_lidar is not None:
        prev_front = get_distance(prev_lidar, FRONT_ANGLE, FRONT_SPREAD)
        if prev_front < 0.2:
            reward += 4
    if prev_lidar is not None and prev_action == 0:
        prev_front = get_distance(prev_lidar, FRONT_ANGLE, FRONT_SPREAD)
        if front_dist > prev_front:
            openness_improvement = front_dist - prev_front
            reward += 1 + (openness_improvement * 3)
    # NEW: Reward maintaining safe rear distance after backward movement
    if prev_action == 3 and back_dist > BACK_SAFE_DISTANCE:
        reward += 2  # Encourage safe backward navigation

    # Odometry-based rewards/penalties
    if prev_position is not None:
        dist_moved = math.sqrt((position.x - prev_position.x)**2 + (position.y - prev_position.y)**2)
        reward += dist_moved * 5  # Reward for covering new ground
        if position.check_loop():
            reward -= 10  # Strong penalty for revisiting positions
            print("🔄 Loop detected! Penalizing...")

    return reward

def maintain_wall_distance(client, lidar):
    """Adjust trajectory to maintain optimal wall distance"""
    right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
    
    if right_dist < IDEAL_WALL_DISTANCE - 0.05:
        correction = (IDEAL_WALL_DISTANCE - right_dist) * WALL_CORRECTION_FACTOR
        move(client, FORWARD_SPEED * 0.5, correction, FORWARD_DURATION * 0.8)
        return True, FORWARD_SPEED * 0.5 * FORWARD_DURATION * 0.8
    elif right_dist > IDEAL_WALL_DISTANCE + 0.1:
        correction = (IDEAL_WALL_DISTANCE - right_dist) * WALL_CORRECTION_FACTOR * 0.7
        move(client, FORWARD_SPEED * 0.5, correction, FORWARD_DURATION * 0.8)
        return True, FORWARD_SPEED * 0.5 * FORWARD_DURATION * 0.8
    return False, 0

def hybrid_rl_wall_follower(client, max_steps=MAX_STEPS):
    """Hybrid RL and wall-following with odometry and back distance"""
    agent = EnhancedRLAgent()
    position = RobotPosition(client)
    steps = 0
    right_turn_counter = 0
    prev_lidar = None
    prev_action = None
    total_reward = 0
    states_visited = set()
    learning_active = False
    
    print("🤖 Starting hybrid RL/wall-follower with odometry and back distance")
    print(f"⏱️ Will begin using learned actions after {STEPS_BEFORE_RL} steps if Q-values > {Q_CONFIDENCE_THRESHOLD}")

    try:
        while steps < max_steps:
            position.update()
            prev_position = copy.deepcopy(position)
            lidar = get_lidar_data_once(client, True)
            
            front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
            if front_dist < 0.12:
                print("⚠️ Emergency backup - too close to wall")
                movement = move_backward_a_bit(client)
                position.update()
                lidar = get_lidar_data_once(client, True)

            state = agent.get_state(lidar, position.orientation)
            states_visited.add(state)

            use_rl_action = False
            max_q_value = np.max(agent.q_table[state])
            if steps > STEPS_BEFORE_RL and max_q_value > Q_CONFIDENCE_THRESHOLD:
                action = agent.get_action(state)
                use_rl_action = True
                if not learning_active:
                    learning_active = True
                    print(f"🎓 LEARNING ACTIVATED! Now using RL actions")
                print(f"🧠 Using learned action: {action} for state {state} (Q-value: {max_q_value:.2f})")
            else:
                if steps > STEPS_BEFORE_RL and max_q_value > 0:
                    print(f"📊 Not using RL yet - state {state} max Q-value ({max_q_value:.2f}) below threshold {Q_CONFIDENCE_THRESHOLD}")
                
                right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
                if is_front_clear(lidar) and right_dist < RIGHT_OPEN_THRESHOLD and right_dist > 0.05:
                    action = 0
                    did_correct, movement = maintain_wall_distance(client, lidar)
                    if did_correct:
                        position.update()
                        new_lidar = get_lidar_data_once(client, True)
                        new_state = agent.get_state(new_lidar, position.orientation)
                        reward = calculate_reward(new_lidar, prev_lidar, 0, position, prev_position)
                        agent.update_q_value(state, 0, reward, new_state)
                        prev_lidar = new_lidar
                        prev_action = 0
                        total_reward += reward
                        steps += 1
                        continue
                elif is_right_clear(lidar) or right_dist > RIGHT_OPEN_THRESHOLD:
                    action = 1
                    right_turn_counter += 1
                elif is_front_clear(lidar):
                    action = 0
                else:
                    action = 2
                    right_turn_counter = 0
                
                if position.check_loop():
                    print("🔄 Breaking potential loop with left turn")
                    action = 2
                    right_turn_counter = 0

            if not use_rl_action or prev_action != action:
                movement = execute_action(client, action)
                position.update()

            new_lidar = get_lidar_data_once(client, True)
            position.update()
            new_state = agent.get_state(new_lidar, position.orientation)
            reward = calculate_reward(new_lidar, prev_lidar, action, position, prev_position)
            total_reward += reward
            agent.update_q_value(state, action, reward, new_state)
            
            prev_lidar = new_lidar
            prev_action = action
            prev_position = copy.deepcopy(position)
            
            if steps % 20 == 0:
                print(f"Step {steps}: Pos ({position.x:.1f}, {position.y:.1f}), Yaw: {position.orientation:.2f}, Action: {action}, Reward: {reward:.1f}")
            
            if steps % 100 == 0 or steps == max_steps - 1:
                coverage = len(states_visited) / agent.num_states * 100
                print(f"🔍 State coverage: {len(states_visited)}/{agent.num_states} states ({coverage:.1f}%)")
                agent.save_q_table()
            
            steps += 1
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("⛔ Stopped by user")
        agent.save_q_table()
        position.odom.unsubscribe()
        coverage = len(states_visited) / agent.num_states * 100
        print(f"🔍 Final state coverage: {len(states_visited)}/{agent.num_states} states ({coverage:.1f}%)")
    
    agent.save_q_table()
    position.odom.unsubscribe()
    print(f"🏁 Run completed: {steps} steps, Total reward: {total_reward:.1f}")
    if learning_active:
        print("🎓 Learning was activated during this run!")
    else:
        print("📚 Learning still building confidence - run longer next time")

def main(client):
    print("🤖 Maze-solving Robot with Enhanced RL, Odometry, and Back Distance")
    print("🧭 Using odometry from /odom topic")
    print("⚙️ Expanded state space (324 states) with orientation")
    print("📏 Using back distance in reward function")
    
    hybrid_rl_wall_follower(client, max_steps=MAX_STEPS)

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