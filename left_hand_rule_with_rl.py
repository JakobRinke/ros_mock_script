import roslibpy
import math
import time
import numpy as np
import os
import random
from heiner_comunication.lidar import get_lidar_data_once, LidarFrame
from heiner_comunication.motor_control import move, rotate

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


# Position tracking class - simplified for Pi
class RobotPosition:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.orientation = 0
        
    def update(self, movement, rotation):
        """Update position based on movement and rotation"""
        self.orientation += rotation
        self.orientation = (self.orientation + math.pi) % (2 * math.pi) - math.pi
        
        self.x += movement * math.cos(self.orientation)
        self.y += movement * math.sin(self.orientation)


# Enhanced RL agent with expanded state space for 4GB Raspberry Pi
class EnhancedRLAgent:
    def __init__(self):
        # Use an expanded state space (3^4 = 81 states)
        # Front, right, left, back - each with 3 possible values
        self.num_states = 81
        self.num_actions = 4
        
        # Initialize Q-table or load existing one
        self.init_q_table()
        
        # Track updates to save computational resources
        self.update_counter = 0
        
    def init_q_table(self):
        """Initialize or load Q-table, handle transition from old format"""
        # Add debug information about file location
        print(f"📁 Q-table file location: {os.path.abspath(Q_TABLE_FILE)}")
        
        # Check if old format exists (8x4 table) vs new format (81x4)
        old_format_exists = False
        if os.path.exists(Q_TABLE_FILE):
            try:
                existing_table = np.load(Q_TABLE_FILE)
                if existing_table.shape == (8, 4):  # Old format
                    old_format_exists = True
                elif existing_table.shape == (self.num_states, self.num_actions):
                    # Already in correct format - add detailed debugging
                    self.q_table = existing_table
                    
                    # Add debug information
                    non_zero_count = np.count_nonzero(self.q_table)
                    max_value = np.max(self.q_table)
                    print(f"✅ Loaded existing Q-table with {non_zero_count} non-zero values")
                    print(f"📊 Maximum Q-value in table: {max_value:.2f}")
                    
                    # Show top 3 learned actions
                    if non_zero_count > 0:
                        flat_indices = np.argsort(self.q_table.flatten())[-3:]
                        for idx in flat_indices:
                            state_idx = idx // self.num_actions
                            action_idx = idx % self.num_actions
                            print(f"💡 Strong learning: State {state_idx}, Action {action_idx}, Q-value: {self.q_table[state_idx, action_idx]:.2f}")
                    return
                else:
                    print(f"⚠️ Unexpected Q-table shape: {existing_table.shape}, creating new one")
                    self.q_table = np.zeros((self.num_states, self.num_actions))
                    return
            except Exception as e:
                print(f"❌ Error inspecting Q-table: {e}")
                self.q_table = np.zeros((self.num_states, self.num_actions))
                return
        
        # If old format exists, back it up and create new
        if old_format_exists:
            try:
                print("🔄 Upgrading from 8-state to 81-state representation")
                # Backup old table
                old_table = np.load(Q_TABLE_FILE)
                np.save(OLD_Q_TABLE_FILE, old_table)
                print(f"📦 Old Q-table backed up to {OLD_Q_TABLE_FILE}")
                
                # Create new table
                self.q_table = np.zeros((self.num_states, self.num_actions))
                
                # Transfer knowledge from old to new where possible
                # Transfer all non-zero entries to maintain learning
                if np.max(old_table) > 0:
                    # Find states with significant learning
                    for old_state in range(8):
                        for action in range(4):
                            if old_table[old_state, action] > 0.1:
                                # Map from old state description to new state ranges
                                if old_state == 4:  # front clear, no right wall
                                    # Map to states with front=2 (clear), right=2 (far)
                                    self._transfer_to_matching_states(old_table[old_state, action], 
                                                                     action, 2, 2, None, None)
                                elif old_state == 6:  # front clear, right wall good
                                    # Map to states with front=2 (clear), right=1 (good)
                                    self._transfer_to_matching_states(old_table[old_state, action], 
                                                                     action, 2, 1, None, None)
                                elif old_state == 0:  # front blocked, no right wall
                                    self._transfer_to_matching_states(old_table[old_state, action], 
                                                                     action, 0, 2, None, None)
                                elif old_state == 2:  # front blocked, right wall present
                                    self._transfer_to_matching_states(old_table[old_state, action], 
                                                                     action, 0, 1, None, None)
                
                print("🧠 Knowledge transferred from old to new state representation")
                
            except Exception as e:
                print(f"❌ Error during Q-table upgrade: {e}")
                self.q_table = np.zeros((self.num_states, self.num_actions))
        else:
            # No existing table
            print("Creating new enhanced Q-table")
            self.q_table = np.zeros((self.num_states, self.num_actions))
    
    def _transfer_to_matching_states(self, value, action, front, right, left, back):
        """Helper to transfer values from old to new state space"""
        # Transfer to all matching states (handling None as wildcard)
        left_range = range(3) if left is None else [left]
        back_range = range(3) if back is None else [back]
        
        for l in left_range:
            for b in back_range:
                new_idx = self.state_index(front, right, l, b)
                # Transfer with 80% strength to avoid overconfidence
                self.q_table[new_idx, action] = value * 0.8
    
    def is_enhanced_table(self, filename):
        """Check if existing table is already in enhanced format"""
        try:
            table = np.load(filename)
            return table.shape[0] == 81  # Enhanced table has 81 states
        except:
            return False
    
    def state_index(self, front, right, left, back):
        """Convert 4 state values to a single index"""
        return front * 27 + right * 9 + left * 3 + back
    
    def get_state(self, lidar):
        """Convert LIDAR data to an enhanced state representation with occasional diagnostics"""
        # Get distances in four directions
        front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
        right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
        left_dist = get_distance(lidar, LEFT_ANGLE, RIGHT_SPREAD)
        back_dist = get_distance(lidar, BACK_ANGLE, FRONT_SPREAD)
        
        # Print raw readings occasionally for diagnostics
        if random.random() < 0.01:  # 1% of the time
            print(f"📏 Raw distances - F:{front_dist:.2f}, R:{right_dist:.2f}, L:{left_dist:.2f}, B:{back_dist:.2f}")
        
        # Categorize each direction into 3 distance levels using consistent thresholds
        # 0: too close (< 0.15m), 1: good distance (0.15-0.3m), 2: far (>0.3m)
        front_state = 0 if front_dist < DIST_CLOSE else 1 if front_dist < DIST_MEDIUM else 2
        right_state = 0 if right_dist < DIST_CLOSE else 1 if right_dist < DIST_MEDIUM else 2
        left_state = 0 if left_dist < DIST_CLOSE else 1 if left_dist < DIST_MEDIUM else 2
        back_state = 0 if back_dist < DIST_CLOSE else 1 if back_dist < DIST_MEDIUM else 2
        
        # Convert 4 features to a single state index (0-80)
        return self.state_index(front_state, right_state, left_state, back_state)
    
    def get_action(self, state):
        """Select action using epsilon-greedy policy"""
        if np.random.random() < EXPLORATION_RATE:
            # Explore: choose random action
            return np.random.randint(0, self.num_actions)
        else:
            # Exploit: choose best action
            return np.argmax(self.q_table[state])
    
    def update_q_value(self, state, action, reward, next_state):
        """Update Q-value - optimized for every-step updates"""
        self.update_counter += 1
        
        # Now updates every step for faster learning
        # Standard Q-learning update rule
        current_q = self.q_table[state, action]
        max_next_q = np.max(self.q_table[next_state])
        new_q = current_q + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max_next_q - current_q)
        self.q_table[state, action] = new_q
        
        # Save Q-table periodically (every 100 updates)
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
def get_distance(lidar:LidarFrame, angle, spread):
    """Get average distance at specified angle"""
    return lidar.get_value_around_angle(angle, spread)


def is_front_clear(lidar):
    """Check if front is clear"""
    return get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD) > DISTANCE_THRESHOLD


def is_right_clear(lidar):
    """Check if right side is clear"""
    return get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD) > RIGHT_OPEN_THRESHOLD


def get_right_opening_angle(lidar):
    """Calculate angle of right opening"""
    start_angle = RIGHT_ANGLE - math.radians(90)
    stop_angle = RIGHT_ANGLE + math.radians(30)
    scan_data = lidar.get_values_between_angles(start_angle, stop_angle)

    if len(scan_data) == 0:
        return -math.radians(45)  # Default if no data

    # Find maximum distance and corresponding angle
    max_distance = 0
    best_angle = RIGHT_ANGLE
    for i, distance in enumerate(scan_data):
        if distance > max_distance:
            max_distance = distance
            angle = start_angle + (stop_angle - start_angle) * i / len(scan_data)
            best_angle = angle

    # Normalize angle
    angle_diff = (best_angle - FRONT_ANGLE + math.pi) % (2 * math.pi) - math.pi
    return angle_diff


# Robot movement primitives
def move_forward(client):
    """Move forward at normal speed"""
    move(client, FORWARD_SPEED, 0, FORWARD_DURATION)
    return FORWARD_SPEED * FORWARD_DURATION


def move_forward_a_bit(client):
    """Move forward slightly"""
    move(client, FORWARD_SPEED * 0.2, 0, FORWARD_DURATION * 0.6)
    return FORWARD_SPEED * 0.2 * FORWARD_DURATION * 0.6


def move_backward(client):
    """Move backward"""
    move(client, -FORWARD_SPEED, 0, FORWARD_DURATION)
    return -FORWARD_SPEED * FORWARD_DURATION


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


def calculate_reward(lidar, prev_lidar=None, prev_action=None):
    """Calculate reward - enhanced for STL-19P TOF LiDAR characteristics"""
    # Get distance readings in all four directions
    front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
    right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
    left_dist = get_distance(lidar, LEFT_ANGLE, RIGHT_SPREAD)
    back_dist = get_distance(lidar, BACK_ANGLE, FRONT_SPREAD)
    
    # Initialize reward
    reward = 0
    
    # Severe penalties for unsafe conditions - adjusted for TOF accuracy
    if front_dist < 0.12:  # Dangerously close to front wall
        reward -= 8  # Increased penalty to strongly discourage
    if right_dist < 0.12:  # Too close to right wall
        reward -= 5  # Increased for TOF sensor
    if left_dist < 0.12:  # Too close to left wall
        reward -= 3
        
    # Rewards for desired behavior - made more significant for faster learning
    if 0.18 < right_dist < 0.28:  # Ideal right wall distance for TOF
        reward += 3  # Increased from 2
        
    # INCREASED REWARD for moving forward in open space
    if front_dist > 0.35 and prev_action == 0:  # Moving forward in open space
        # Progressive reward based on available space
        space_reward = 3  # Base reward (increased from 2)
        if front_dist > 0.5:  # Extra reward for very open spaces
            space_reward += 2
        if front_dist > 0.7:  # Even more reward for extremely open spaces
            space_reward += 2
        reward += space_reward
        
    if right_dist < RIGHT_OPEN_THRESHOLD and right_dist > 0.18 and front_dist > 0.35:
        reward += 2  # Good wall-following position
        
    # Reward for making a good right turn when there's an opening
    if prev_action == 1 and right_dist > 0.4:
        reward += 3  # Increased from 2
    
    # Reward for successfully navigating a corner
    if prev_action == 2 and front_dist > 0.35 and prev_lidar is not None:
        prev_front = get_distance(prev_lidar, FRONT_ANGLE, FRONT_SPREAD)
        if prev_front < 0.2:  # Was blocked, now clear after left turn
            reward += 4  # Increased from 2
    
    # Reward for efficient path finding
    if prev_lidar is not None and prev_action == 0:
        prev_front = get_distance(prev_lidar, FRONT_ANGLE, FRONT_SPREAD)
        if front_dist > prev_front:  # Moving toward more open space
            # Increased reward for finding significantly more open space
            openness_improvement = front_dist - prev_front
            reward += 1 + (openness_improvement * 3)  # Base 1 plus proportional bonus
    
    return reward


def maintain_wall_distance(client, lidar):
    """Adjust trajectory to maintain optimal wall distance"""
    right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
    
    # Only correct if we're significantly off the ideal distance
    if right_dist < IDEAL_WALL_DISTANCE - 0.05:
        # Gentle correction away from wall
        correction = (IDEAL_WALL_DISTANCE - right_dist) * WALL_CORRECTION_FACTOR
        move(client, FORWARD_SPEED * 0.5, correction, FORWARD_DURATION * 0.8)
        return True, FORWARD_SPEED * 0.5 * FORWARD_DURATION * 0.8  # Return movement amount
    elif right_dist > IDEAL_WALL_DISTANCE + 0.1:
        # Gentle correction toward wall
        correction = (IDEAL_WALL_DISTANCE - right_dist) * WALL_CORRECTION_FACTOR * 0.7
        move(client, FORWARD_SPEED * 0.5, correction, FORWARD_DURATION * 0.8)
        return True, FORWARD_SPEED * 0.5 * FORWARD_DURATION * 0.8  # Return movement amount
    return False, 0


def hybrid_rl_wall_follower(client, max_steps=MAX_STEPS):
    """Hybrid approach combining enhanced RL and wall-following - FIXED VERSION"""
    # Initialize enhanced agent and position tracking
    agent = EnhancedRLAgent()
    position = RobotPosition()
    
    # Variables for tracking
    steps = 0
    right_turn_counter = 0
    prev_lidar = None
    prev_action = None
    total_reward = 0
    states_visited = set()  # Track state coverage
    learning_active = False  # Track if learning has started using RL actions
    
    print("🤖 Starting hybrid RL/wall-follower with enhanced state space")
    print("🧠 Agent now tracking 81 different states for improved learning")
    print(f"⏱️ Will begin using learned actions after {STEPS_BEFORE_RL} steps if Q-values > {Q_CONFIDENCE_THRESHOLD}")
    
    try:
        while steps < max_steps:
            # Get LIDAR data
            lidar = get_lidar_data_once(client, True)
            
            # Safety check - emergency backup if too close to wall
            front_dist = get_distance(lidar, FRONT_ANGLE, FRONT_SPREAD)
            if front_dist < 0.12:
                print("⚠️ Emergency backup - too close to wall")
                movement = move_backward_a_bit(client)
                position.update(movement, 0)
                lidar = get_lidar_data_once(client, True)
                
            # Get current state
            state = agent.get_state(lidar)
            states_visited.add(state)  # Track unique states visited
            
            # Get action - FIXED to use global variables instead of hardcoded values
            use_rl_action = False
            max_q_value = np.max(agent.q_table[state])
            
            # Check if we should use RL based on steps and confidence threshold
            if steps > STEPS_BEFORE_RL and max_q_value > Q_CONFIDENCE_THRESHOLD:
                # Use RL action if we've learned something useful
                action = agent.get_action(state)
                use_rl_action = True
                
                # First time using learning? Announce it
                if not learning_active:
                    learning_active = True
                    print(f"🎓 LEARNING ACTIVATED! Now using RL actions when confident")
                    
                print(f"🧠 Using learned action: {action} for state {state} (Q-value: {max_q_value:.2f})")
            else:
                # Provide debug info about why not using RL if after steps threshold
                if steps > STEPS_BEFORE_RL and max_q_value > 0:
                    print(f"📊 Not using RL yet - state {state} max Q-value ({max_q_value:.2f}) below threshold {Q_CONFIDENCE_THRESHOLD}")
                
                # Default to wall-following logic to build initial experiences
                # Check right wall status
                right_dist = get_distance(lidar, RIGHT_ANGLE, RIGHT_SPREAD)
                
                if is_front_clear(lidar) and right_dist < RIGHT_OPEN_THRESHOLD and right_dist > 0.05:
                    # Right wall exists and front is clear
                    action = 0  # Move forward
                    # Apply wall distance correction if needed
                    did_correct, movement = maintain_wall_distance(client, lidar)
                    if did_correct:
                        # Update position and skip to next iteration as we already moved
                        position.update(movement, 0)
                        
                        # Still update Q-values even though we used a special movement
                        new_lidar = get_lidar_data_once(client, True)
                        new_state = agent.get_state(new_lidar)
                        reward = calculate_reward(new_lidar, prev_lidar, 0)  # Treat as forward action
                        agent.update_q_value(state, 0, reward, new_state)
                        
                        # Update for next iteration
                        prev_lidar = new_lidar
                        prev_action = 0
                        steps += 1
                        total_reward += reward
                        continue
                        
                elif is_right_clear(lidar) or right_dist > RIGHT_OPEN_THRESHOLD:
                    # Right is open, should turn right
                    action = 1  # Turn right
                    right_turn_counter += 1
                elif is_front_clear(lidar):
                    # Front clear but right wall not ideal
                    action = 0  # Move forward
                else:
                    # Front blocked, turn left
                    action = 2  # Turn left
                    right_turn_counter = 0
                    
                # Reset right turn counter if too many consecutive right turns
                if right_turn_counter > MAX_RIGHT_TURNS:
                    print("🔄 Breaking potential loop with left turn")
                    action = 2  # Force left turn
                    right_turn_counter = 0
            
            # Execute the selected action
            if not use_rl_action or prev_action != action:
                # Only execute if it's an RL action or a different action than before
                movement = execute_action(client, action)
                
                # Update position tracking
                if action == 0 or action == 3:  # Forward or backward
                    position.update(movement, 0)
                else:  # Turning
                    position.update(0, movement)
            
            # Get new state
            new_lidar = get_lidar_data_once(client, True)
            new_state = agent.get_state(new_lidar)
            
            # Calculate reward
            reward = calculate_reward(new_lidar, prev_lidar, action)
            total_reward += reward
            
            # Update Q-values (now updates every step)
            agent.update_q_value(state, action, reward, new_state)
            
            # Update for next iteration
            prev_lidar = new_lidar
            prev_action = action
            
            # Print status - less frequently to reduce output volume
            if steps % 20 == 0:  # Print every 20 steps instead of 10
                print(f"Step {steps}: Pos ({position.x:.1f}, {position.y:.1f}), Action: {action}, Reward: {reward:.1f}")
                
            # Print state coverage periodically and save Q-table
            if steps % 100 == 0 or steps == max_steps - 1:
                coverage = len(states_visited) / agent.num_states * 100
                print(f"🔍 State coverage: {len(states_visited)}/{agent.num_states} states ({coverage:.1f}%)")
                # Save Q-table at regular intervals
                agent.save_q_table()
            
            steps += 1
            time.sleep(0.05)  # Brief pause
            
    except KeyboardInterrupt:
        print("⛔ Stopped by user")
        agent.save_q_table()
        
        # Print final statistics
        coverage = len(states_visited) / agent.num_states * 100
        print(f"🔍 State coverage: {len(states_visited)}/{agent.num_states} states ({coverage:.1f}%)")
    
    # Final save
    agent.save_q_table()
    print(f"🏁 Run completed: {steps} steps, Total reward: {total_reward:.1f}")
    
    # Print final statistics
    coverage = len(states_visited) / agent.num_states * 100
    print(f"🔍 State coverage: {len(states_visited)}/{agent.num_states} states ({coverage:.1f}%)")
    if learning_active:
        print("🎓 Learning was activated during this run!")
    else:
        print("📚 Learning still building confidence - run longer next time to activate learning")


def main(client):
    """Main function to start the maze-solving robot"""
    print("🤖 Maze-solving Robot with Enhanced RL - Optimized for STL-19P LiDAR")
    print("🧭 Starting at position (0,0)")
    print("⚙️ Using expanded state space (81 states) for 4GB Raspberry Pi 5")
    print("⌛ Optimized for shorter runs - will use learned actions after 200 steps")

    # Force learning mode
    FORCE_LEARNING = False  # Set to True to force using learned behaviors
    
    if FORCE_LEARNING:
        print("⚡ FORCE LEARNING MODE ENABLED - Will use learned actions immediately")
        # Override normal parameters
        global STEPS_BEFORE_RL
        global Q_CONFIDENCE_THRESHOLD
        STEPS_BEFORE_RL = 0  # Start using learned actions immediately
        Q_CONFIDENCE_THRESHOLD = 0.1  # Use very low threshold for demonstration
    
    # Run the hybrid algorithm with reasonable step limit
    hybrid_rl_wall_follower(client, max_steps=MAX_STEPS)


# --- Entry point ---
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