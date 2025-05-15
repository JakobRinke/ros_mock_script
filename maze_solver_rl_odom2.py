import roslibpy
import math
import time
import numpy as np
import os
import random
import copy
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

# Enhanced distance discretization
DIST_VERY_CLOSE = 0.12
DIST_CLOSE = 0.18
DIST_MEDIUM = 0.35
DIST_FAR = 0.60
DIST_VERY_FAR = 0.90

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

# Fallback mechanism parameters
REWARD_WINDOW_SIZE = 10
POOR_PERFORMANCE_THRESHOLD = -3.0
FALLBACK_DURATION = 15  # steps

# Loop detection
MAX_RIGHT_TURNS = 3

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

    def odom_callback(self, message):
        """Callback to process odometry data"""
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
        except KeyError as e:
            print(f"❌ Odometry message missing field: {e}")

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
        self.visited_positions = []
        self.position_threshold = 0.1

    def update(self):
        """Update position from odometry data"""
        self.x = self.odom.x
        self.y = self.odom.y
        self.orientation = self.odom.yaw
        self.orientation = (self.orientation + math.pi) % (2 * math.pi) - math.pi
        self.visited_positions.append((self.x, self.y))

    def check_loop(self):
        """Check if robot is revisiting a position"""
        current_pos = (self.x, self.y)
        for pos in self.visited_positions[:-10]:
            dist = math.sqrt((current_pos[0] - pos[0])**2 + (current_pos[1] - pos[1])**2)
            if dist < self.position_threshold:
                return True
        return False

# Enhanced RL Agent with Orientation
class EnhancedRLAgent:
    def __init__(self):
        # Updated for 5 distance bins instead of 3
        self.num_distance_bins = 5
        self.num_orientations = 4
        self.num_states = self.num_distance_bins**4 * self.num_orientations  # 5^4 * 4 = 2500 states
        self.num_actions = 4
        self.init_q_table()
        print(f"🧠 Enhanced state space: {self.num_states} states with {self.num_distance_bins} distance bins")

    def state_index(self, front, right, left, back, orientation):
        orientation_bin = self.discretize_orientation(orientation)
        # Updated indexing formula for 5 bins per direction
        return (front * self.num_distance_bins**3 * self.num_orientations + 
                right * self.num_distance_bins**2 * self.num_orientations + 
                left * self.num_distance_bins * self.num_orientations + 
                back * self.num_orientations + 
                orientation_bin)

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
        
        # Enhanced 5-level discretization
        front_state = (0 if front_dist < DIST_VERY_CLOSE else 
                      1 if front_dist < DIST_CLOSE else 
                      2 if front_dist < DIST_MEDIUM else 
                      3 if front_dist < DIST_FAR else 4)
        
        right_state = (0 if right_dist < DIST_VERY_CLOSE else 
                      1 if right_dist < DIST_CLOSE else 
                      2 if right_dist < DIST_MEDIUM else 
                      3 if right_dist < DIST_FAR else 4)
        
        left_state = (0 if left_dist < DIST_VERY_CLOSE else 
                     1 if left_dist < DIST_CLOSE else 
                     2 if left_dist < DIST_MEDIUM else 
                     3 if left_dist < DIST_FAR else 4)
        
        back_state = (0 if back_dist < DIST_VERY_CLOSE else 
                     1 if back_dist < DIST_CLOSE else 
                     2 if back_dist < DIST_MEDIUM else 
                     3 if back_dist < DIST_FAR else 4)
        
        return self.state_index(front_state, right_state, left_state, back_state, orientation)

    def init_q_table(self):
        print(f"📁 Q-table file location: {os.path.abspath(Q_TABLE_FILE)}")
        if os.path.exists(Q_TABLE_FILE):
            try:
                existing_table = np.load(Q_TABLE_FILE)
                
                # Handle different possible existing Q-table shapes
                if existing_table.shape == (81, 4):  # Original 3^4 states without orientation
                    print("🔄 Upgrading Q-table to enhanced discretization with orientation")
                    np.save(OLD_Q_TABLE_FILE, existing_table)
                    self.q_table = np.zeros((self.num_states, self.num_actions), dtype=np.float32)
                    # Transfer knowledge where possible (complex mapping)
                    # This is simplified - a more advanced mapping would be better
                    print("🧠 Basic knowledge transfer - previous knowledge will influence exploration")
                    
                elif existing_table.shape == (324, 4):  # Previous 3^4 * 4 states with orientation
                    print("🔄 Upgrading Q-table from 3-bin to 5-bin discretization")
                    np.save(OLD_Q_TABLE_FILE, existing_table)
                    self.q_table = np.zeros((self.num_states, self.num_actions), dtype=np.float32)
                    
                    # Attempt to map old states to new states where possible
                    print("⚙️ Transferring knowledge from old state space to new state space")
                    for old_front in range(3):
                        for old_right in range(3):
                            for old_left in range(3):
                                for old_back in range(3):
                                    for old_orient in range(4):
                                        # Map old 3-bin states to new 5-bin states (approximate)
                                        new_front = min(4, old_front * 2)
                                        new_right = min(4, old_right * 2)
                                        new_left = min(4, old_left * 2)
                                        new_back = min(4, old_back * 2)
                                        
                                        # Calculate old and new indices
                                        old_idx = (old_front * 27 * 4 + old_right * 9 * 4 + 
                                                  old_left * 3 * 4 + old_back * 4 + old_orient)
                                        
                                        new_idx = (new_front * self.num_distance_bins**3 * self.num_orientations + 
                                                  new_right * self.num_distance_bins**2 * self.num_orientations + 
                                                  new_left * self.num_distance_bins * self.num_orientations + 
                                                  new_back * self.num_orientations + old_orient)
                                        
                                        # Transfer Q-values with some decay
                                        for action in range(4):
                                            self.q_table[new_idx, action] = existing_table[old_idx, action] * 0.8
                    
                    print("🧠 Knowledge transferred to new state space")
                    
                elif existing_table.shape == (self.num_states, self.num_actions):
                    # Convert to float32 if not already
                    self.q_table = existing_table.astype(np.float32)
                    print(f"✅ Loaded compatible Q-table with {np.count_nonzero(self.q_table)} non-zero values")
                    return
                else:
                    print(f"⚠️ Unexpected Q-table shape: {existing_table.shape}")
                    self.q_table = np.zeros((self.num_states, self.num_actions), dtype=np.float32)
            except Exception as e:
                print(f"❌ Error inspecting Q-table: {e}")
                self.q_table = np.zeros((self.num_states, self.num_actions), dtype=np.float32)
        else:
            print("Creating new enhanced Q-table with 5-bin discretization")
            self.q_table = np.zeros((self.num_states, self.num_actions), dtype=np.float32)

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
    
    if front_dist < 0.12:
        reward -= 8
    if right_dist < 0.12:
        reward -= 5
    if left_dist < 0.12:
        reward -= 3
    if back_dist < 0.12:
        reward -= 4
        print("⚠️ Too close to rear obstacle, penalty applied")
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
    if prev_action == 3 and back_dist > 0.35:
        reward += 2
        print("✅ Safe backward movement, reward added")

    if prev_position is not None:
        dist_moved = math.sqrt((position.x - prev_position.x)**2 + (position.y - prev_position.y)**2)
        reward += dist_moved * 5
        if position.check_loop():
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
    
    # Fallback mechanism variables
    recent_rewards = []
    fallback_mode = False
    fallback_steps = 0
    
    print("🤖 Starting hybrid RL/wall-follower with improved state space and fallback mechanism")
    print(f"⏱️ Will begin using learned actions after {STEPS_BEFORE_RL} steps if Q-values > {Q_CONFIDENCE_THRESHOLD}")
    print(f"🔄 Fallback mechanism active: will revert to wall-following if performance drops below {POOR_PERFORMANCE_THRESHOLD}")

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
            
            # Update fallback mechanism
            if len(recent_rewards) >= 5:
                avg_recent_reward = sum(recent_rewards[-5:]) / 5
                if avg_recent_reward < POOR_PERFORMANCE_THRESHOLD and not fallback_mode:
                    fallback_mode = True
                    fallback_steps = FALLBACK_DURATION
                    print(f"📉 Poor performance detected (avg reward: {avg_recent_reward:.2f}) - falling back to wall-following")
            
            if fallback_mode:
                fallback_steps -= 1
                if fallback_steps <= 0:
                    fallback_mode = False
                    print("⬆️ Exiting fallback mode - trying learned policy again")
            
            use_rl_action = False
            
            # Decision logic with fallback mechanism
            if fallback_mode:
                # Use wall-following logic
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
                        
                        # Update recent rewards for fallback tracking
                        recent_rewards.append(reward)
                        if len(recent_rewards) > REWARD_WINDOW_SIZE:
                            recent_rewards.pop(0)
                            
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
                    
                print(f"🔙 Fallback mode: using wall-following action {action}")
            else:
                # Try to use RL-based action if appropriate
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
                    
                    # Fall back to wall-following logic
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
                            
                            # Update recent rewards for fallback tracking
                            recent_rewards.append(reward)
                            if len(recent_rewards) > REWARD_WINDOW_SIZE:
                                recent_rewards.pop(0)
                                
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
            
            # Update recent rewards for fallback tracking
            recent_rewards.append(reward)
            if len(recent_rewards) > REWARD_WINDOW_SIZE:
                recent_rewards.pop(0)
            
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
    print("🤖 Maze-solving Robot with Improved RL and Fallback Mechanism")
    print("🧭 Using odometry from /odom topic")
    print("⚙️ 5-level distance discretization (2,500 states)")
    print("🔄 Fallback mechanism active for more robust navigation")
    print("💾 Using memory-optimized storage (float32)")
    
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