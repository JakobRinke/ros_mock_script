import roslibpy
import math
import time
import numpy as np
import os
import random
import datetime
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
BACK_ANGLE = math.pi
DISTANCE_THRESHOLD = 0.20
RIGHT_OPEN_THRESHOLD = 0.30
IDEAL_WALL_DISTANCE = 0.2
WALL_CORRECTION_FACTOR = 0.6

# Reinforcement Learning parameters
LEARNING_RATE = 0.2
DISCOUNT_FACTOR = 0.8
EXPLORATION_RATE = 0.2
UPDATE_INTERVAL = 1
STEPS_BEFORE_RL = 200
Q_CONFIDENCE_THRESHOLD = 0.7
MAX_STEPS = 2000
Q_TABLE_FILE = "q_table.npy"
OLD_Q_TABLE_FILE = "q_table_old.npy"

# Loop detection
MAX_RIGHT_TURNS = 3

# State space parameters
DIST_CLOSE = 0.18
DIST_MEDIUM = 0.35

# Logging parameters
VERBOSE_ODOM_LOGGING = True  # Set to True to log detailed odometry data
ODOM_LOG_INTERVAL = 10       # Log odometry every X steps
LOG_FILE = f"robot_odom_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

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
        # Additional data for debugging
        self.linear_vel_x = 0.0
        self.angular_vel_z = 0.0
        self.raw_quaternion = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        self.seq = 0
        self.frame_id = ""
        self.data_valid = False
        self.max_stale_time = 1.0  # Maximum time (seconds) before data is considered stale
        
    def odom_callback(self, message):
        """Callback to process odometry data"""
        try:
            self.x = message['pose']['pose']['position']['x']
            self.y = message['pose']['pose']['position']['y']
            orientation_q = message['pose']['pose']['orientation']
            self.raw_quaternion = orientation_q.copy()
            self.yaw = quaternion_to_yaw(
                orientation_q['x'],
                orientation_q['y'],
                orientation_q['z'],
                orientation_q['w']
            )
            self.last_update = time.time()
            self.data_valid = True
            
            # Store additional data
            if 'twist' in message and 'twist' in message['twist']:
                if 'linear' in message['twist']['twist']:
                    self.linear_vel_x = message['twist']['twist']['linear']['x']
                if 'angular' in message['twist']['twist']:
                    self.angular_vel_z = message['twist']['twist']['angular']['z']
            
            if 'header' in message:
                if 'seq' in message['header']:
                    self.seq = message['header']['seq']
                if 'frame_id' in message['header']:
                    self.frame_id = message['header']['frame_id']
                    
            if VERBOSE_ODOM_LOGGING and random.random() < 0.05:  # Log approximately 5% of messages
                self.log_odom_data("Callback")
                
        except KeyError as e:
            print(f"❌ Odometry message missing field: {e}")
            self.data_valid = False
    
    def is_data_fresh(self):
        """Check if odometry data is fresh"""
        return self.data_valid and (time.time() - self.last_update) < self.max_stale_time
    
    def log_odom_data(self, context=""):
        """Log detailed odometry data for debugging"""
        yaw_degrees = math.degrees(self.yaw)
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        data_status = "FRESH" if self.is_data_fresh() else "STALE"
        log_msg = (
            f"[{timestamp}] ODOM {context} ({data_status}): "
            f"Pos({self.x:.3f}, {self.y:.3f}), "
            f"Yaw: {self.yaw:.3f}rad ({yaw_degrees:.1f}°), "
            f"Vel: lin={self.linear_vel_x:.2f}, ang={self.angular_vel_z:.2f}, "
            f"Seq: {self.seq}"
        )
        print(log_msg)
        
        # Also log to file if needed
        if LOG_FILE:
            try:
                with open(LOG_FILE, "a") as f:
                    f.write(log_msg + "\n")
            except Exception as e:
                print(f"❌ Error writing to log file: {e}")

    def subscribe(self, client):
        """Subscribe to /odom topic"""
        self.odom_sub = roslibpy.Topic(client, '/odom', 'nav_msgs/Odometry')
        self.odom_sub.subscribe(self.odom_callback)
        print("📍 Subscribed to /odom topic")
        print(f"📝 Logging odometry data to {LOG_FILE}" if LOG_FILE else "")

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
        self.visited_positions = []
        self.position_threshold = 0.1
        self.total_distance = 0.0
        self.last_position = (0.0, 0.0)
        self.max_positions = 1000  # Maximum number of positions to store
        
        # Create log file header
        if LOG_FILE:
            try:
                with open(LOG_FILE, "w") as f:
                    f.write("Timestamp,Step,X,Y,Orientation,Action,Reward,FrontDist,RightDist,LeftDist,BackDist\n")
            except Exception as e:
                print(f"❌ Error creating log file: {e}")

    def update(self):
        """Update position from odometry data and return whether data is fresh"""
        # Check if odometry data is fresh
        if not self.odom.is_data_fresh():
            print("⚠️ Odometry data is stale! Last update was "
                  f"{time.time() - self.odom.last_update:.1f} seconds ago.")
            return False
            
        new_x = self.odom.x
        new_y = self.odom.y
        new_orientation = self.odom.yaw
        
        # Calculate distance traveled since last update
        if hasattr(self, 'last_position'):
            dx = new_x - self.x
            dy = new_y - self.y
            step_distance = math.sqrt(dx*dx + dy*dy)
            
            # Filter out tiny movements (likely noise)
            if step_distance > 0.005:  # Minimum movement threshold (5mm)
                self.total_distance += step_distance
                
                # Log significant movements
                if step_distance > 0.01 and VERBOSE_ODOM_LOGGING:
                    print(f"🚶 Movement: ({self.x:.3f}, {self.y:.3f}) → ({new_x:.3f}, {new_y:.3f}), "
                          f"Δ={step_distance:.3f}m, Total={self.total_distance:.2f}m")
        
        self.x = new_x
        self.y = new_y
        self.orientation = new_orientation
        # Normalize orientation to [-pi, pi]
        self.orientation = (self.orientation + math.pi) % (2 * math.pi) - math.pi
        
        # Store position and limit list size to prevent memory issues
        self.visited_positions.append((self.x, self.y))
        if len(self.visited_positions) > self.max_positions:
            self.visited_positions = self.visited_positions[-self.max_positions:]
            
        return True
        
    def log_position_data(self, step, action=None, reward=None, lidar=None):
        """Log detailed position data with distances if available"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        orientation_deg = math.degrees(self.orientation)
        
        # Basic position info
        log_msg = (
            f"[{timestamp}] Step {step}: "
            f"Pos({self.x:.3f}, {self.y:.3f}), "
            f"Orient: {self.orientation:.2f}rad ({orientation_deg:.1f}°)"
        )
        
        # Add action and reward if available
        if action is not None:
            action_names = ["FORWARD", "RIGHT", "LEFT", "BACK"]
            action_name = action_names[action] if 0 <= action < len(action_names) else f"Unknown({action})"
            log_msg += f", Action: {action_name}"
        
        if reward is not None:
            log_msg += f", Reward: {reward:.2f}"
            
        # Add distances if lidar data is available
        if lidar is not None:
            front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
            right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
            left_dist = get_distance(lidar, LEFT_ANGLE, RIGHT_SPREAD)
            back_dist = get_distance(lidar, BACK_ANGLE, FRONT_SPREAD)
            log_msg += f", Dist[F:{front_dist:.2f}, R:{right_dist:.2f}, L:{left_dist:.2f}, B:{back_dist:.2f}]"
            
            # Log to file
            if LOG_FILE:
                try:
                    with open(LOG_FILE, "a") as f:
                        f.write(f"{timestamp},{step},{self.x:.3f},{self.y:.3f},{self.orientation:.3f},"
                                f"{action if action is not None else ''},"
                                f"{reward if reward is not None else ''},"
                                f"{front_dist:.3f},{right_dist:.3f},{left_dist:.3f},{back_dist:.3f}\n")
                except Exception as e:
                    print(f"❌ Error writing to log file: {e}")
        
        print(log_msg)

    def check_loop(self):
        """Check if robot is revisiting a position"""
        current_pos = (self.x, self.y)
        for pos in self.visited_positions[:-10]:
            dist = math.sqrt((current_pos[0] - pos[0])**2 + (current_pos[1] - pos[1])**2)
            if dist < self.position_threshold:
                return True
        return False

# Custom Position object for copying without pickling issues
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
        print(f"📁 Q-table file location: {os.path.abspath(Q_TABLE_FILE)}")
        if os.path.exists(Q_TABLE_FILE):
            try:
                existing_table = np.load(Q_TABLE_FILE)
                if existing_table.shape == (self.num_states, self.num_actions):
                    self.q_table = existing_table
                    print(f"✅ Loaded Q-table with {np.count_nonzero(self.q_table)} non-zero values")
                elif existing_table.shape == (81, 4):
                    print("🔄 Upgrading Q-table to include orientation")
                    np.save(OLD_Q_TABLE_FILE, existing_table)
                    self.q_table = np.zeros((self.num_states, self.num_actions))
                    for state in range(81):
                        for action in range(4):
                            for orient in range(4):
                                new_state = state * 4 + orient
                                self.q_table[new_state, action] = existing_table[state, action] * 0.8
                    print("🧠 Knowledge transferred")
                else:
                    print(f"⚠️ Unexpected Q-table shape: {existing_table.shape}, creating new")
                    self.q_table = np.zeros((self.num_states, self.num_actions))
            except Exception as e:
                print(f"❌ Error inspecting Q-table: {e}")
                self.q_table = np.zeros((self.num_states, self.num_actions))
        else:
            print("Creating new Q-table")
            self.q_table = np.zeros((self.num_states, self.num_actions))

    def get_action(self, state):
        if np.random.random() < EXPLORATION_RATE:
            return np.random.randint(0, self.num_actions)
        else:
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
    return lidar.get_value_around_angle(angle, spread)

def is_front_clear(lidar):
    return get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD) > DISTANCE_THRESHOLD

def is_right_clear(lidar):
    return get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD) > RIGHT_OPEN_THRESHOLD

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
    left_dist = get_distance(lidar, LEFT_ANGLE, RIGHT_SPREAD)
    back_dist = get_distance(lidar, BACK_ANGLE, FRONT_SPREAD)
    
    reward = 0
    
    # Safety penalties for being too close to obstacles
    if front_dist < 0.12:
        reward -= 8
    if right_dist < 0.12:
        reward -= 5
    if left_dist < 0.12:
        reward -= 3
    if back_dist < 0.12:
        reward -= 4
        print("⚠️ Too close to rear obstacle, penalty applied")
        
    # Reward for maintaining ideal wall distance
    if 0.18 < right_dist < 0.28:
        reward += 3
        
    # Reward for open space ahead
    if front_dist > 0.35 and prev_action == 0:
        space_reward = 3
        if front_dist > 0.5:
            space_reward += 2
        if front_dist > 0.7:
            space_reward += 2
        reward += space_reward
        
    # Wall-following rewards
    if right_dist < RIGHT_OPEN_THRESHOLD and right_dist > 0.18 and front_dist > 0.35:
        reward += 2
        
    # Turn effectiveness rewards
    if prev_action == 1 and right_dist > 0.4:
        reward += 3
    if prev_action == 2 and front_dist > 0.35 and prev_lidar is not None:
        prev_front = get_distance(prev_lidar, FRONT_ANGLE, FRONT_SPREAD)
        if prev_front < 0.2:
            reward += 4
            
    # Reward for opening up space ahead
    if prev_lidar is not None and prev_action == 0:
        prev_front = get_distance(prev_lidar, FRONT_ANGLE, FRONT_SPREAD)
        if front_dist > prev_front:
            openness_improvement = front_dist - prev_front
            reward += 1 + (openness_improvement * 3)
            
    # Reward for safe backward movement
    if prev_action == 3 and back_dist > 0.35:
        reward += 2
        print("✅ Safe backward movement, reward added")

    # Movement-based rewards
    if prev_position is not None:
        # Calculate distance moved (ignoring very small movements as noise)
        dist_moved = math.sqrt((position.x - prev_position.x)**2 + (position.y - prev_position.y)**2)
        # Filter out tiny movements (likely odometry noise)
        if dist_moved > 0.01:  # Only reward significant movement
            reward += dist_moved * 5
        
        # Loop detection penalty
        if hasattr(position, 'check_loop') and position.check_loop():
            reward -= 10
            print("🔄 Loop detected! Penalizing...")

    return reward

def maintain_wall_distance(client, lidar):
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
    agent = EnhancedRLAgent()
    position = RobotPosition(client)
    steps = 0
    right_turn_counter = 0
    prev_lidar = None
    prev_action = None
    total_reward = 0
    states_visited = set()
    learning_active = False
    prev_position = None
    stale_data_retries = 0
    
    print("🤖 Starting hybrid RL/wall-follower with odometry")
    print(f"⏱️ Will begin using learned actions after {STEPS_BEFORE_RL} steps if Q-values > {Q_CONFIDENCE_THRESHOLD}")
    print(f"📝 Odometry logging is {'ENABLED' if VERBOSE_ODOM_LOGGING else 'disabled'}")

    try:
        while steps < max_steps:
            # Update position with freshness check
            position_updated = position.update()
            if not position_updated:
                stale_data_retries += 1
                if stale_data_retries > 5:
                    print("❌ Too many stale data retries, stopping")
                    break
                print(f"⏳ Waiting for fresh data (retry {stale_data_retries}/5)...")
                time.sleep(0.2)
                continue
            else:
                stale_data_retries = 0
            
            # Create a safe copy of position data without using deepcopy
            prev_position = PositionData(position.x, position.y, position.orientation)
            
            # Get lidar data and validate it
            lidar = get_lidar_data_once(client, True)
            
            # Check for invalid LiDAR data
            try:
                front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
                right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
                left_dist = get_distance(lidar, LEFT_ANGLE, RIGHT_SPREAD)
                back_dist = get_distance(lidar, BACK_ANGLE, FRONT_SPREAD)
            except Exception as e:
                print(f"❌ LiDAR data validation error: {e}")
                time.sleep(0.2)
                continue
            
            # Log odometry data periodically
            if VERBOSE_ODOM_LOGGING and steps % ODOM_LOG_INTERVAL == 0:
                position.log_position_data(steps, prev_action, None, lidar)
                
            # Emergency backup if too close to obstacle
            if front_dist < 0.12:
                print("⚠️ Emergency backup - too close to wall")
                movement = move_backward_a_bit(client)
                position.update()
                lidar = get_lidar_data_once(client, True)

            # Get state from current sensory data
            state = agent.get_state(lidar, position.orientation)
            states_visited.add(state)

            # Decide whether to use RL or rule-based action
            use_rl_action = False
            max_q_value = np.max(agent.q_table[state])
            
            if steps > STEPS_BEFORE_RL and max_q_value > Q_CONFIDENCE_THRESHOLD:
                # Use RL-based action
                action = agent.get_action(state)
                use_rl_action = True
                if not learning_active:
                    learning_active = True
                    print(f"🎓 LEARNING ACTIVATED! Now using RL actions")
                print(f"🧠 Using learned action: {action} for state {state} (Q-value: {max_q_value:.2f})")
            else:
                # Use rule-based action (wall-following)
                if steps > STEPS_BEFORE_RL and max_q_value > 0:
                    print(f"📊 Not using RL yet - state {state} max Q-value ({max_q_value:.2f}) below threshold {Q_CONFIDENCE_THRESHOLD}")
                
                if is_front_clear(lidar) and right_dist < RIGHT_OPEN_THRESHOLD and right_dist > 0.05:
                    action = 0
                    did_correct, movement = maintain_wall_distance(client, lidar)
                    if did_correct:
                        # If we made a wall correction, process it as an action
                        position_updated = position.update()
                        if not position_updated:
                            # Skip if odometry is stale
                            print("⚠️ Stale odometry after wall correction, skipping update")
                            continue
                        
                        new_lidar = get_lidar_data_once(client, True)
                        new_state = agent.get_state(new_lidar, position.orientation)
                        reward = calculate_reward(new_lidar, prev_lidar, 0, position, prev_position)
                        agent.update_q_value(state, 0, reward, new_state)
                        prev_lidar = new_lidar
                        prev_action = 0
                        total_reward += reward
                        steps += 1
                        
                        # Log state transitions for debugging
                        if steps % 10 == 0:
                            print(f"🔄 State: {state} → {new_state}, Action: WALL_CORRECTION, Reward: {reward:.1f}")
                        
                        continue
                elif is_right_clear(lidar) or right_dist > RIGHT_OPEN_THRESHOLD:
                    action = 1
                    right_turn_counter += 1
                    
                    # Check if too many consecutive right turns
                    if right_turn_counter > MAX_RIGHT_TURNS:
                        print("🔄 Too many consecutive right turns, forcing left turn")
                        action = 2
                        right_turn_counter = 0
                elif is_front_clear(lidar):
                    action = 0
                else:
                    action = 2
                    right_turn_counter = 0
                
                # Check for loop using position history
                if position.check_loop():
                    print("🔄 Breaking potential loop with left turn")
                    action = 2
                    right_turn_counter = 0

            # Execute the selected action
            if not use_rl_action or prev_action != action:
                movement = execute_action(client, action)
                position_updated = position.update()
                if not position_updated:
                    print("⚠️ Stale odometry after action execution, continuing anyway")
                    # We continue anyway since we already performed the action

            # Get new state and calculate reward
            new_lidar = get_lidar_data_once(client, True)
            position.update()  # Update position again after action
            new_state = agent.get_state(new_lidar, position.orientation)
            
            # Calculate reward based on new state and action taken
            reward = calculate_reward(new_lidar, prev_lidar, action, position, prev_position)
            total_reward += reward
            
            # Update Q-table with new experience
            agent.update_q_value(state, action, reward, new_state)
            
            # Store current state for next iteration
            prev_lidar = new_lidar
            prev_action = action
            prev_position = PositionData(position.x, position.y, position.orientation)
            
            # Log detailed position data after action
            if VERBOSE_ODOM_LOGGING:
                position.log_position_data(steps, action, reward, new_lidar)
            
            # Log state transitions periodically
            if steps % 10 == 0:
                action_names = ["FORWARD", "RIGHT", "LEFT", "BACK"]
                action_name = action_names[action] if 0 <= action < len(action_names) else f"Unknown({action})"
                print(f"🔄 State: {state} → {new_state}, Action: {action_name}, Reward: {reward:.1f}")
            elif steps % 20 == 0:
                # Less verbose logging if VERBOSE_ODOM_LOGGING is disabled
                print(f"Step {steps}: Pos ({position.x:.1f}, {position.y:.1f}), Yaw: {position.orientation:.2f}, Action: {action}, Reward: {reward:.1f}")
            
            # Periodically save Q-table and log coverage
            if steps % 100 == 0 or steps == max_steps - 1:
                coverage = len(states_visited) / agent.num_states * 100
                print(f"🔍 State coverage: {len(states_visited)}/{agent.num_states} states ({coverage:.1f}%)")
                agent.save_q_table()
                
                # Log odometry handler data directly
                if VERBOSE_ODOM_LOGGING:
                    position.odom.log_odom_data("Periodic Check")
            
            steps += 1
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("⛔ Stopped by user")
        agent.save_q_table()
        position.odom.unsubscribe()
        coverage = len(states_visited) / agent.num_states * 100
        print(f"🔍 Final state coverage: {len(states_visited)}/{agent.num_states} states ({coverage:.1f}%)")
    except Exception as e:
        print(f"❌ Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    
    # Final cleanup and reporting
    agent.save_q_table()
    position.odom.unsubscribe()
    print(f"🏁 Run completed: {steps} steps, Total reward: {total_reward:.1f}")
    print(f"📏 Total distance traveled: {position.total_distance:.2f}m")
    if learning_active:
        print("🎓 Learning was activated during this run!")
    else:
        print("📚 Learning still building confidence - run longer next time")

def main(client):
    print("🤖 Maze-solving Robot with Enhanced RL and Odometry")
    print("🧭 Using odometry from /odom topic")
    print("⚙️ Expanded state space (324 states) with orientation")
    print("📡 Using back distance in reward function for safer navigation")
    print(f"📊 Odometry logging to file: {LOG_FILE}")
    
    # Print extended odometry data at startup
    initial_odom = OdometryHandler()
    initial_odom.subscribe(client)
    time.sleep(1)  # Wait for odometry to start
    print("\n=== INITIAL ODOMETRY DATA ===")
    initial_odom.log_odom_data("Startup")
    initial_odom.unsubscribe()
    
    hybrid_rl_wall_follower(client, max_steps=MAX_STEPS)

# Entry point
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