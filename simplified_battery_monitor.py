import roslibpy
import time
import json
import os
from datetime import datetime

class BatteryMonitor:
    def __init__(self, client, log_file="battery_log.json"):
        """
        Initialize battery monitor with logging and rule-based parameter control.
        
        Args:
            host (str): ROS bridge host
            port (int): ROS bridge port
            log_file (str): File to log battery data
        """
        self.client = client
        self.log_file = log_file
        
        # ROS connections
        self.client = None
        self.battery_sub = None
        self.power_mode_pub = None
        
        # Voltage thresholds for 2S LiPo
        self.MIN_VOLTAGE = 6.0          # absolute minimum
        self.CRITICAL_VOLTAGE = 6.6     # 3.3V/cell
        self.CONSERVATIVE_VOLTAGE = 7.0
        self.EFFICIENT_VOLTAGE = 7.6
        self.MAX_VOLTAGE = 8.4          # fully charged (4.2V/cell)
        
        # Current state
        self.current_mode = "NORMAL"
        self.current_voltage = 7.4      # Default assumption
        self.battery_percentage = 100.0
        self.is_solving_maze = False    # Flag for maze solving mode
        
        # Battery logging
        self.battery_logs = []
        self.last_log_time = 0
        self.log_interval = 5           # Log every 5 seconds
        
        # Rule-based parameters for each power mode
        self.mode_params = {
            "NORMAL": {
                "speed_factor": 1.0,    # 100% of base speed
                "Kp_side": 2.5,         # Original PID gains
                "Kp_front": 1.5,
                "smoothing": 0.2,       # Low smoothing
                "max_angular": 1.5,     # Full angular velocity
                "sampling_rate": 0.05   # 20Hz sampling
            },
            "EFFICIENT": {
                "speed_factor": 0.8,    # 80% of base speed
                "Kp_side": 2.2,         # Slightly reduced gains
                "Kp_front": 1.3,
                "smoothing": 0.3,       # More smoothing
                "max_angular": 1.2,     # Reduced angular velocity
                "sampling_rate": 0.06   # ~16Hz sampling
            },
            "CONSERVATIVE": {
                "speed_factor": 0.6,    # 60% of base speed
                "Kp_side": 1.8,         # Further reduced gains
                "Kp_front": 1.0,
                "smoothing": 0.4,       # Even more smoothing
                "max_angular": 1.0,     # Limited angular velocity
                "sampling_rate": 0.08   # 12.5Hz sampling
            },
            "CRITICAL": {
                "speed_factor": 0.4,    # 40% of base speed
                "Kp_side": 1.5,         # Minimal gains
                "Kp_front": 0.8,
                "smoothing": 0.5,       # Maximum smoothing
                "max_angular": 0.7,     # Very limited turning
                "sampling_rate": 0.1    # 10Hz sampling
            }
        }
        
        # Load previous logs if available
        self._load_logs()
        
    def connect(self):
        """
        Connect to ROS bridge and set up subscribers/publishers.
        
        Returns:
            bool: True if connection successful, False otherwise
        """  
        try:
            # Subscribe to battery voltage topic
            self.battery_sub = roslibpy.Topic(
                self.client,
                '/ros_robot_controller/battery',
                'std_msgs/UInt16'
            )
            self.battery_sub.subscribe(self.battery_callback)
            
            # Publisher for power mode
            self.power_mode_pub = roslibpy.Topic(
                self.client,
                '/power_mode',
                'std_msgs/String'
            )
            self.power_mode_pub.advertise()
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def battery_callback(self, message):
        """
        Process battery voltage messages, log data, and update power mode.
        
        Args:
            message (dict): ROS message containing battery voltage
        """
        # Convert from millivolts to volts
        voltage_mv = message['data']
        voltage = voltage_mv / 1000.0
        self.current_voltage = voltage
        
        # Calculate percentage
        self.battery_percentage = self.convert_voltage_to_percentage(voltage)
        
        # Determine power mode
        new_mode = self.determine_power_mode(voltage)
        
        # Log status
        print(f"Voltage: {voltage:.2f} V, Battery: {self.battery_percentage:.1f}%, Mode: {new_mode}")
        
        # Log data with timestamp
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.log_battery_data(voltage)
            self.last_log_time = current_time
        
        # Update mode if changed
        if new_mode != self.current_mode:
            old_mode = self.current_mode
            self.current_mode = new_mode
            
            if self.power_mode_pub:
                self.power_mode_pub.publish(roslibpy.Message({'data': new_mode}))
                print(f"Power mode changed: {old_mode} -> {new_mode}")
                
                # Print the new parameters
                params = self.get_current_params()
                print(f"New parameters: Speed: {params['speed_factor']*100:.0f}%, " + 
                      f"Smoothing: {params['smoothing']}, Max Angular: {params['max_angular']}")
    
    def convert_voltage_to_percentage(self, voltage):
        """
        Convert battery voltage to percentage.
        
        Args:
            voltage (float): Battery voltage in volts
            
        Returns:
            float: Battery percentage (0-100)
        """
        # Clamp voltage to realistic LiPo range
        clamped_voltage = max(self.MIN_VOLTAGE, min(self.MAX_VOLTAGE, voltage))
        percentage = ((clamped_voltage - self.MIN_VOLTAGE) / 
                      (self.MAX_VOLTAGE - self.MIN_VOLTAGE)) * 100.0
        return round(percentage, 1)
    
    def determine_power_mode(self, voltage):
        """
        Determine appropriate power mode based on voltage.
        
        Args:
            voltage (float): Battery voltage in volts
            
        Returns:
            str: Power mode ("NORMAL", "EFFICIENT", "CONSERVATIVE", or "CRITICAL")
        """
        if voltage >= self.EFFICIENT_VOLTAGE:
            return "NORMAL"
        elif voltage >= self.CONSERVATIVE_VOLTAGE:
            return "EFFICIENT"
        elif voltage >= self.CRITICAL_VOLTAGE:
            return "CONSERVATIVE"
        else:
            return "CRITICAL"
    
    def get_current_params(self):
        """
        Get parameters for the current power mode.
        
        Returns:
            dict: Parameters for the current power mode
        """
        return self.mode_params[self.current_mode]
    
    def log_battery_data(self, voltage):
        """
        Log battery data with timestamp.
        
        Args:
            voltage (float): Current battery voltage
        """
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        log_entry = {
            "timestamp": timestamp,
            "unix_time": time.time(),
            "voltage": round(voltage, 3),
            "percentage": self.battery_percentage,
            "power_mode": self.current_mode,
            "is_solving_maze": self.is_solving_maze
        }
        
        self.battery_logs.append(log_entry)
        self._save_logs()
        
        print(f"Battery data logged: {voltage:.2f}V, {self.battery_percentage:.1f}%")
    
    def _load_logs(self):
        """Load battery logs from file if it exists."""
        try:
            if os.path.exists(self.log_file):
                with open(self.log_file, 'r') as f:
                    self.battery_logs = json.load(f)
                print(f"Loaded {len(self.battery_logs)} battery log entries")
        except Exception as e:
            print(f"Error loading battery logs: {e}")
            self.battery_logs = []
    
    def _save_logs(self):
        """Save battery logs to file."""
        try:
            with open(self.log_file, 'w') as f:
                json.dump(self.battery_logs, f, indent=2)
        except Exception as e:
            print(f"Error saving battery logs: {e}")
    
    def set_maze_solving_mode(self, is_solving=True):
        """
        Set whether the robot is currently solving a maze.
        This flag is stored in the logs for analysis.
        
        Args:
            is_solving (bool): Whether the robot is solving a maze
        """
        self.is_solving_maze = is_solving
        print(f"Maze solving mode: {'Enabled' if is_solving else 'Disabled'}")
    
    def get_battery_status(self):
        """
        Get current battery status.
        
        Returns:
            dict: Current battery status information
        """
        return {
            'voltage': self.current_voltage,
            'percentage': self.battery_percentage,
            'power_mode': self.current_mode,
            'is_solving_maze': self.is_solving_maze
        }
    
    def apply_energy_rules(self, base_speed, Kp_side_original=2.5, Kp_front_original=1.5):
        """
        Get energy-optimized parameters based on current power mode.
        Use these parameters for navigation to implement adaptive behavior.
        
        Args:
            base_speed (float): Base speed before power mode adjustments
            Kp_side_original (float, optional): Original side PID gain
            Kp_front_original (float, optional): Original front PID gain
            
        Returns:
            dict: Parameters adjusted for the current power mode
        """
        params = self.get_current_params()
        
        # Scale the speed by the mode's speed factor
        adjusted_speed = base_speed * params["speed_factor"]
        
        # Scale the PID gains if custom originals provided
        if Kp_side_original != 2.5 or Kp_front_original != 1.5:
            # Calculate ratios compared to default values
            side_ratio = Kp_side_original / 2.5
            front_ratio = Kp_front_original / 1.5
            
            # Apply ratios to the mode's values
            Kp_side = params["Kp_side"] * side_ratio
            Kp_front = params["Kp_front"] * front_ratio
        else:
            # Use the mode's values directly
            Kp_side = params["Kp_side"]
            Kp_front = params["Kp_front"]
        
       # Return all relevant parameters, including speed_factor for diagnostics
        return {
            'speed': adjusted_speed,
            'speed_factor': params['speed_factor'],  # Added this to fix the diagnostic function
            'Kp_side': Kp_side,
            'Kp_front': Kp_front,
            'smoothing': params['smoothing'],
            'max_angular': params['max_angular'],
            'sampling_rate': params['sampling_rate'],
            'power_mode': self.current_mode
        }
    
    def test_rule_based_system(self, base_speed=0.5, print_details=True):
        """
        Test the rule-based system with different voltage levels.
        
        Args:
            base_speed (float): Base speed to use for testing
            print_details (bool): Whether to print detailed results
            
        Returns:
            dict: Test results for each power mode
        """
        test_voltages = {
            "NORMAL": 8.0,      # Normal mode
            "EFFICIENT": 7.3,    # Efficient mode
            "CONSERVATIVE": 6.8, # Conservative mode
            "CRITICAL": 6.4      # Critical mode
        }
        
        results = {}
        original_voltage = self.current_voltage
        original_mode = self.current_mode
        
        if print_details:
            print("\nTesting rule-based system with base speed:", base_speed)
            print("-" * 50)
        
        for expected_mode, voltage in test_voltages.items():
            # Update state without triggering callbacks
            self.current_voltage = voltage
            self.battery_percentage = self.convert_voltage_to_percentage(voltage)
            self.current_mode = self.determine_power_mode(voltage)
            
            # Get the parameters for this voltage level
            params = self.apply_energy_rules(base_speed)
            
            # Store results
            results[expected_mode] = {
                'voltage': voltage,
                'battery_percentage': self.battery_percentage,
                'power_mode': self.current_mode,
                'speed': params['speed'],
                'speed_factor': params['speed'] / base_speed,
                'Kp_side': params['Kp_side'],
                'Kp_front': params['Kp_front'],
                'smoothing': params['smoothing'],
                'max_angular': params['max_angular'],
                'sampling_rate': params['sampling_rate'],
                'sampling_hz': 1 / params['sampling_rate']
            }
            
            if print_details:
                print(f"Mode: {expected_mode}")
                print(f"  Voltage: {voltage:.2f}V, Battery: {self.battery_percentage:.1f}%")
                print(f"  Speed: {params['speed']:.2f} m/s ({params['speed']/base_speed*100:.0f}% of base)")
                print(f"  Kp values: {params['Kp_side']:.2f}, {params['Kp_front']:.2f}")
                print(f"  Smoothing: {params['smoothing']:.2f}")
                print(f"  Max Angular: {params['max_angular']:.2f}")
                print(f"  Sampling Rate: {params['sampling_rate']:.3f}s ({1/params['sampling_rate']:.1f}Hz)")
                print("")
        
        # Restore original state
        self.current_voltage = original_voltage
        self.battery_percentage = self.convert_voltage_to_percentage(original_voltage)
        self.current_mode = original_mode
        
        return results
    
    def simulate_voltage_drop(self, starting_voltage=8.4, ending_voltage=6.0, steps=10, 
                             base_speed=0.5, print_details=True):
        """
        Simulate a voltage drop and see how the rule-based system responds.
        
        Args:
            starting_voltage (float): Starting voltage
            ending_voltage (float): Ending voltage
            steps (int): Number of steps to simulate
            base_speed (float): Base speed for parameter calculation
            print_details (bool): Whether to print detailed results
            
        Returns:
            list: Results at each step
        """
        voltage_step = (starting_voltage - ending_voltage) / (steps - 1)
        voltages = [starting_voltage - i * voltage_step for i in range(steps)]
        
        results = []
        original_voltage = self.current_voltage
        original_mode = self.current_mode
        
        if print_details:
            print(f"\nSimulating voltage drop from {starting_voltage:.2f}V to {ending_voltage:.2f}V")
            print("-" * 50)
        
        for voltage in voltages:
            # Update state without triggering callbacks
            self.current_voltage = voltage
            self.battery_percentage = self.convert_voltage_to_percentage(voltage)
            self.current_mode = self.determine_power_mode(voltage)
            
            # Get the parameters for this voltage level
            params = self.apply_energy_rules(base_speed)
            
            # Store results
            result = {
                'voltage': voltage,
                'battery_percentage': self.battery_percentage,
                'power_mode': self.current_mode,
                'speed': params['speed'],
                'speed_factor': params['speed'] / base_speed,
                'Kp_side': params['Kp_side'],
                'Kp_front': params['Kp_front'],
                'smoothing': params['smoothing'],
                'max_angular': params['max_angular'],
                'sampling_rate': params['sampling_rate']
            }
            results.append(result)
            
            if print_details:
                print(f"Voltage: {voltage:.2f}V, Battery: {self.battery_percentage:.1f}%, Mode: {self.current_mode}")
                print(f"  Speed: {params['speed']:.2f} m/s ({params['speed']/base_speed*100:.0f}% of base)")
                print(f"  Kp values: {params['Kp_side']:.2f}, {params['Kp_front']:.2f}")
                print(f"  Smoothing: {params['smoothing']:.2f}")
                print(f"  Max Angular: {params['max_angular']:.2f}")
                print(f"  Sampling Rate: {params['sampling_rate']:.3f}s ({1/params['sampling_rate']:.1f}Hz)")
                print("")
        
        # Restore original state
        self.current_voltage = original_voltage
        self.battery_percentage = self.convert_voltage_to_percentage(original_voltage)
        self.current_mode = original_mode
        
        return results
    
    def get_logs_by_maze_solving(self, solving_only=True):
        """
        Get battery logs filtered by maze solving state.
        
        Args:
            solving_only (bool): If True, return only logs during maze solving
            
        Returns:
            list: Filtered battery logs
        """
        if solving_only:
            return [log for log in self.battery_logs if log.get('is_solving_maze', False)]
        else:
            return [log for log in self.battery_logs if not log.get('is_solving_maze', False)]
    
    def get_logs_by_timeframe(self, start_time, end_time):
        """
        Get battery logs within a specific timeframe.
        
        Args:
            start_time (float): Start time (Unix timestamp)
            end_time (float): End time (Unix timestamp)
            
        Returns:
            list: Filtered battery logs
        """
        return [log for log in self.battery_logs 
                if log.get('unix_time', 0) >= start_time and 
                log.get('unix_time', 0) <= end_time]
    
    def get_battery_usage_stats(self, logs=None):
        """
        Calculate battery usage statistics.
        
        Args:
            logs (list, optional): List of log entries to analyze
            
        Returns:
            dict: Battery usage statistics
        """
        if logs is None:
            logs = self.battery_logs
        
        if not logs:
            return {
                'count': 0,
                'avg_voltage': 0,
                'min_voltage': 0,
                'max_voltage': 0,
                'avg_percentage': 0,
                'voltage_drop_per_hour': 0
            }
        
        # Extract data
        voltages = [log.get('voltage', 0) for log in logs]
        percentages = [log.get('percentage', 0) for log in logs]
        times = [log.get('unix_time', 0) for log in logs]
        
        # Basic statistics
        min_voltage = min(voltages)
        max_voltage = max(voltages)
        avg_voltage = sum(voltages) / len(voltages)
        avg_percentage = sum(percentages) / len(percentages)
        
        # Calculate voltage drop per hour if we have enough data
        voltage_drop_per_hour = 0
        if len(voltages) >= 2 and (times[-1] - times[0]) > 0:
            duration_hours = (times[-1] - times[0]) / 3600
            if duration_hours > 0:
                voltage_drop = voltages[0] - voltages[-1]
                voltage_drop_per_hour = voltage_drop / duration_hours
        
        return {
            'count': len(logs),
            'avg_voltage': round(avg_voltage, 3),
            'min_voltage': round(min_voltage, 3),
            'max_voltage': round(max_voltage, 3),
            'avg_percentage': round(avg_percentage, 1),
            'voltage_drop_per_hour': round(voltage_drop_per_hour, 3)
        }
    
    def run(self):
        """
        Run the battery monitor as a standalone service.
        """
        try:
            print("Battery Monitor running. Press Ctrl+C to exit.")
            while self.client and self.client.is_connected:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Battery Monitor stopped by user.")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        print("Cleaning up resources...")
        
        # Unsubscribe from topics
        if self.battery_sub:
            self.battery_sub.unsubscribe()
        
        # Unadvertise publishers
        if self.power_mode_pub:
            self.power_mode_pub.unadvertise()
        
        # Terminate ROS client
        if self.client and self.client.is_connected:
            self.client.terminate()


    def run_live_diagnostics(self, duration=60, test_movement=False, 
                          speed_tests=True, rotation_tests=True, 
                          speed_levels=3, rotation_levels=3,
                          max_speed=0.6, max_rotation=2.0,
                          test_omnidirectional=True, test_bidirectional=True):
        """
        Connect to ROS and perform live diagnostic testing of the rule-based system.
        This function connects to the actual robot and tests with real battery readings.
        Configured for the MentorPi M1 with Mecanum-wheel chassis capabilities.
        
        Args:
            duration (int): How long to run diagnostics (seconds)
            test_movement (bool): Whether to test with movements
            speed_tests (bool): Whether to test different forward speeds
            rotation_tests (bool): Whether to test different rotation rates
            speed_levels (int): Number of speed levels to test per direction
            rotation_levels (int): Number of rotation levels to test per direction
            max_speed (float): Maximum speed to test (m/s) [MentorPi M1: -0.6 to 0.6 m/s]
            max_rotation (float): Maximum rotation speed to test (rad/s) [MentorPi M1: -2 to 2 rad/s]
            test_omnidirectional (bool): Whether to test sideways movement (Y axis)
            test_bidirectional (bool): Whether to test both positive and negative directions
            
        Returns:
            dict: Diagnostic results
        """
        if not self.client or not self.client.is_connected:
            if not self.connect():
                print("Failed to connect to ROS bridge. Cannot run diagnostics.")
                return None
        
        print(f"Starting live diagnostic test for {duration} seconds...")
        print("Monitoring battery voltage and rule-based parameters...")
        print(f"Using MentorPi M1 specifications: {-max_speed:.1f} to {max_speed:.1f} m/s linear, {-max_rotation:.1f} to {max_rotation:.1f} rad/s angular")
        
        # Results storage
        results = {
            'voltage_readings': [],
            'power_modes': [],
            'parameters': [],
            'timestamps': [],
            'movement_tests': []
        }
        
        # Set up cmd_vel publisher if testing movement
        cmd_vel_publisher = None
        if test_movement:
            cmd_vel_publisher = roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist')
            cmd_vel_publisher.advertise()
            print("Movement testing enabled")
            
            # Generate test speeds (positive and negative if bidirectional)
            positive_speeds = [max_speed * (i+1)/speed_levels for i in range(speed_levels)]
            if test_bidirectional:
                negative_speeds = [-s for s in positive_speeds]
                test_speeds = negative_speeds + [0] + positive_speeds  # Include zero
                print(f"Will test {len(test_speeds)} speeds: {[round(s,2) for s in test_speeds]} m/s")
            else:
                test_speeds = positive_speeds
                print(f"Will test {len(test_speeds)} forward speeds: {[round(s,2) for s in test_speeds]} m/s")
            
            # Generate test rotation rates (positive and negative if bidirectional)
            positive_rotations = [max_rotation * (i+1)/rotation_levels for i in range(rotation_levels)]
            if test_bidirectional:
                negative_rotations = [-r for r in positive_rotations]
                test_rotations = negative_rotations + [0] + positive_rotations  # Include zero
                print(f"Will test {len(test_rotations)} rotation rates: {[round(r,2) for r in test_rotations]} rad/s")
            else:
                test_rotations = positive_rotations
                print(f"Will test {len(test_rotations)} rotation rates: {[round(r,2) for r in test_rotations]} rad/s")
        
        try:
            start_time = time.time()
            last_param_print = 0
            last_movement_time = 0
            movement_interval = 10  # seconds between movement test sequences
            
            # For movement testing
            test_phase = 0  # 0=X dir, 1=Y dir, 2=rotation, 3=combined
            
            # Main diagnostic loop
            while time.time() - start_time < duration:
                current_time = time.time()
                
                # Store current state
                results['voltage_readings'].append(self.current_voltage)
                results['power_modes'].append(self.current_mode)
                results['timestamps'].append(current_time)
                
                # Get parameters based on current battery state
                params = self.apply_energy_rules(max_speed)
                results['parameters'].append(params)
                
                # Print diagnostic info every 5 seconds
                if current_time - last_param_print >= 5:
                    print(f"\nTime: {int(current_time - start_time)}s")
                    print(f"Voltage: {self.current_voltage:.2f}V, Battery: {self.battery_percentage:.1f}%")
                    print(f"Power Mode: {self.current_mode}")
                    print(f"  Speed factor: {params['speed_factor']:.2f}")
                    print(f"  Max speed: {params['speed']:.2f} m/s ({params['speed']/max_speed*100:.0f}% of base)")
                    print(f"  Max angular: {params['max_angular']:.2f} rad/s ({params['max_angular']/max_rotation*100:.0f}% of max)")
                    print(f"  Kp side/front: {params['Kp_side']:.2f}/{params['Kp_front']:.2f}")
                    print(f"  Smoothing: {params['smoothing']:.2f}")
                    last_param_print = current_time
                
                # Test with movements if enabled
                if test_movement and cmd_vel_publisher and current_time - last_movement_time >= movement_interval:
                    # Get current rule-based limits
                    max_rule_speed = params['speed']
                    max_rule_angular = params['max_angular']
                    
                    # Display test information
                    print("\nStarting movement test sequence...")
                    print(f"Current mode: {self.current_mode}")
                    print(f"Rule-based limits: Speed={max_rule_speed:.2f}m/s, Angular={max_rule_angular:.2f}rad/s")
                    
                    if test_phase == 0:  # Test X-axis speeds (forward/backward)
                        if speed_tests:
                            # Test each speed level
                            for i, speed in enumerate(test_speeds):
                                # Apply rule-based limit while preserving direction
                                direction = 1 if speed >= 0 else -1
                                actual_speed = min(abs(speed), max_rule_speed) * direction
                                
                                # Skip the zero speed test to save time
                                if abs(actual_speed) < 0.01:
                                    continue
                                
                                direction_text = "forward" if speed >= 0 else "backward"
                                print(f"Testing X direction {direction_text}: {actual_speed:.2f}m/s")
                                
                                # Create command
                                cmd = {
                                    'linear': {'x': actual_speed, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                                }
                                
                                # Record test details
                                test_result = {
                                    'type': 'linear_x',
                                    'direction': direction_text,
                                    'commanded': speed,
                                    'actual': actual_speed,
                                    'limited_by_rules': abs(speed) > max_rule_speed,
                                    'power_mode': self.current_mode,
                                    'voltage': self.current_voltage,
                                    'time': current_time - start_time
                                }
                                results['movement_tests'].append(test_result)
                                
                                # Execute movement
                                cmd_vel_publisher.publish(roslibpy.Message(cmd))
                                time.sleep(1.2)  # Run for 1.2 seconds
                                
                                # Stop
                                stop_cmd = {
                                    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                                }
                                cmd_vel_publisher.publish(roslibpy.Message(stop_cmd))
                                time.sleep(0.3)  # Pause between tests
                        
                        # Move to next phase
                        test_phase = 1 if test_omnidirectional else 2
                    
                    elif test_phase == 1:  # Test Y-axis speeds (left/right) - Mecanum wheels capability
                        if speed_tests and test_omnidirectional:
                            # Test each speed level sideways
                            for i, speed in enumerate(test_speeds):
                                # Apply rule-based limit while preserving direction
                                direction = 1 if speed >= 0 else -1
                                actual_speed = min(abs(speed), max_rule_speed) * direction
                                
                                # Skip the zero speed test to save time
                                if abs(actual_speed) < 0.01:
                                    continue
                                
                                direction_text = "right" if speed >= 0 else "left"
                                print(f"Testing Y direction {direction_text}: {actual_speed:.2f}m/s")
                                
                                # Create command (sideways movement)
                                cmd = {
                                    'linear': {'x': 0.0, 'y': actual_speed, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                                }
                                
                                # Record test details
                                test_result = {
                                    'type': 'linear_y',
                                    'direction': direction_text,
                                    'commanded': speed,
                                    'actual': actual_speed,
                                    'limited_by_rules': abs(speed) > max_rule_speed,
                                    'power_mode': self.current_mode,
                                    'voltage': self.current_voltage,
                                    'time': current_time - start_time
                                }
                                results['movement_tests'].append(test_result)
                                
                                # Execute movement
                                cmd_vel_publisher.publish(roslibpy.Message(cmd))
                                time.sleep(1.2)  # Run for 1.2 seconds
                                
                                # Stop
                                stop_cmd = {
                                    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                                }
                                cmd_vel_publisher.publish(roslibpy.Message(stop_cmd))
                                time.sleep(0.3)  # Pause between tests
                        
                        test_phase = 2  # Move to rotation tests
                    
                    elif test_phase == 2:  # Test rotation speeds
                        if rotation_tests:
                            # Test each rotation level
                            for i, rotation in enumerate(test_rotations):
                                # Apply rule-based limit while preserving direction
                                direction = 1 if rotation >= 0 else -1
                                actual_rotation = min(abs(rotation), max_rule_angular) * direction
                                
                                # Skip the zero rotation test to save time
                                if abs(actual_rotation) < 0.01:
                                    continue
                                
                                direction_text = "counterclockwise" if rotation >= 0 else "clockwise"
                                print(f"Testing rotation {direction_text}: {actual_rotation:.2f}rad/s")
                                
                                # Create command
                                cmd = {
                                    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': actual_rotation}
                                }
                                
                                # Record test details
                                test_result = {
                                    'type': 'angular',
                                    'direction': direction_text,
                                    'commanded': rotation,
                                    'actual': actual_rotation,
                                    'limited_by_rules': abs(rotation) > max_rule_angular,
                                    'power_mode': self.current_mode,
                                    'voltage': self.current_voltage,
                                    'time': current_time - start_time
                                }
                                results['movement_tests'].append(test_result)
                                
                                # Execute movement
                                cmd_vel_publisher.publish(roslibpy.Message(cmd))
                                time.sleep(1.2)  # Run for 1.2 seconds
                                
                                # Stop
                                stop_cmd = {
                                    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                                }
                                cmd_vel_publisher.publish(roslibpy.Message(stop_cmd))
                                time.sleep(0.3)  # Pause between tests
                        
                        test_phase = 3  # Move to combined tests
                    
                    elif test_phase == 3:  # Test combined movement
                        if speed_tests and rotation_tests:
                            # Use a moderate speed and rotation for combined tests
                            # Choose a positive speed and rotation for the first test
                            if test_bidirectional:
                                speed = positive_speeds[0]
                                rotation = positive_rotations[0]
                            else:
                                speed = test_speeds[0]
                                rotation = test_rotations[0]
                            
                            # Apply rule-based limits
                            actual_speed = min(speed, max_rule_speed)
                            actual_rotation = min(rotation, max_rule_angular)
                            
                            print(f"Testing combined X+rotation: Speed={actual_speed:.2f}m/s (forward), Rotation={actual_rotation:.2f}rad/s (counterclockwise)")
                            
                            # Create command
                            cmd = {
                                'linear': {'x': actual_speed, 'y': 0.0, 'z': 0.0},
                                'angular': {'x': 0.0, 'y': 0.0, 'z': actual_rotation}
                            }
                            
                            # Record test details
                            test_result = {
                                'type': 'combined_x',
                                'direction': 'forward+ccw',
                                'commanded_speed': speed,
                                'actual_speed': actual_speed,
                                'commanded_rotation': rotation,
                                'actual_rotation': actual_rotation,
                                'limited_by_rules': speed > max_rule_speed or rotation > max_rule_angular,
                                'power_mode': self.current_mode,
                                'voltage': self.current_voltage,
                                'time': current_time - start_time
                            }
                            results['movement_tests'].append(test_result)
                            
                            # Execute movement
                            cmd_vel_publisher.publish(roslibpy.Message(cmd))
                            time.sleep(1.5)  # Run for 1.5 seconds
                            
                            # Stop
                            stop_cmd = {
                                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                            }
                            cmd_vel_publisher.publish(roslibpy.Message(stop_cmd))
                            time.sleep(0.5)  # Pause
                            
                            # If bidirectional, test the opposite direction
                            if test_bidirectional:
                                # Choose a negative speed and rotation for the second test
                                speed = negative_speeds[0]
                                rotation = negative_rotations[0]
                                
                                # Apply rule-based limits preserving direction
                                actual_speed = -min(abs(speed), max_rule_speed)
                                actual_rotation = -min(abs(rotation), max_rule_angular)
                                
                                print(f"Testing combined X+rotation: Speed={actual_speed:.2f}m/s (backward), Rotation={actual_rotation:.2f}rad/s (clockwise)")
                                
                                # Create command
                                cmd = {
                                    'linear': {'x': actual_speed, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': actual_rotation}
                                }
                                
                                # Record test details
                                test_result = {
                                    'type': 'combined_x',
                                    'direction': 'backward+cw',
                                    'commanded_speed': speed,
                                    'actual_speed': actual_speed,
                                    'commanded_rotation': rotation,
                                    'actual_rotation': actual_rotation,
                                    'limited_by_rules': abs(speed) > max_rule_speed or abs(rotation) > max_rule_angular,
                                    'power_mode': self.current_mode,
                                    'voltage': self.current_voltage,
                                    'time': current_time - start_time
                                }
                                results['movement_tests'].append(test_result)
                                
                                # Execute movement
                                cmd_vel_publisher.publish(roslibpy.Message(cmd))
                                time.sleep(1.5)  # Run for 1.5 seconds
                                
                                # Stop
                                stop_cmd = {
                                    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                                }
                                cmd_vel_publisher.publish(roslibpy.Message(stop_cmd))
                                time.sleep(0.5)  # Pause
                            
                            # If omnidirectional, also test Y + rotation
                            if test_omnidirectional:
                                # Reset to a positive speed and rotation
                                if test_bidirectional:
                                    speed = positive_speeds[0]
                                    rotation = positive_rotations[0]
                                else:
                                    speed = test_speeds[0]
                                    rotation = test_rotations[0]
                                
                                # Apply rule-based limits
                                actual_speed = min(speed, max_rule_speed)
                                actual_rotation = min(rotation, max_rule_angular)
                                
                                print(f"Testing combined Y+rotation: Speed={actual_speed:.2f}m/s (right), Rotation={actual_rotation:.2f}rad/s (counterclockwise)")
                                
                                # Create command (Y direction + rotation)
                                cmd = {
                                    'linear': {'x': 0.0, 'y': actual_speed, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': actual_rotation}
                                }
                                
                                # Record test details
                                test_result = {
                                    'type': 'combined_y',
                                    'direction': 'right+ccw',
                                    'commanded_speed': speed,
                                    'actual_speed': actual_speed,
                                    'commanded_rotation': rotation,
                                    'actual_rotation': actual_rotation,
                                    'limited_by_rules': speed > max_rule_speed or rotation > max_rule_angular,
                                    'power_mode': self.current_mode,
                                    'voltage': self.current_voltage,
                                    'time': current_time - start_time
                                }
                                results['movement_tests'].append(test_result)
                                
                                # Execute movement
                                cmd_vel_publisher.publish(roslibpy.Message(cmd))
                                time.sleep(1.5)  # Run for 1.5 seconds
                                
                                # Stop
                                stop_cmd = {
                                    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                                }
                                cmd_vel_publisher.publish(roslibpy.Message(stop_cmd))
                                time.sleep(0.5)  # Pause
                                
                                # If bidirectional, test the opposite Y direction
                                if test_bidirectional:
                                    # Negative speed and rotation
                                    speed = negative_speeds[0]
                                    rotation = negative_rotations[0]
                                    
                                    # Apply rule-based limits preserving direction
                                    actual_speed = -min(abs(speed), max_rule_speed)
                                    actual_rotation = -min(abs(rotation), max_rule_angular)
                                    
                                    print(f"Testing combined Y+rotation: Speed={actual_speed:.2f}m/s (left), Rotation={actual_rotation:.2f}rad/s (clockwise)")
                                    
                                    # Create command
                                    cmd = {
                                        'linear': {'x': 0.0, 'y': actual_speed, 'z': 0.0},
                                        'angular': {'x': 0.0, 'y': 0.0, 'z': actual_rotation}
                                    }
                                    
                                    # Record test details
                                    test_result = {
                                        'type': 'combined_y',
                                        'direction': 'left+cw',
                                        'commanded_speed': speed,
                                        'actual_speed': actual_speed,
                                        'commanded_rotation': rotation,
                                        'actual_rotation': actual_rotation,
                                        'limited_by_rules': abs(speed) > max_rule_speed or abs(rotation) > max_rule_angular,
                                        'power_mode': self.current_mode,
                                        'voltage': self.current_voltage,
                                        'time': current_time - start_time
                                    }
                                    results['movement_tests'].append(test_result)
                                    
                                    # Execute movement
                                    cmd_vel_publisher.publish(roslibpy.Message(cmd))
                                    time.sleep(1.5)  # Run for 1.5 seconds
                                    
                                    # Stop
                                    stop_cmd = {
                                        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                                    }
                                    cmd_vel_publisher.publish(roslibpy.Message(stop_cmd))
                                    time.sleep(0.5)  # Pause
                        
                        test_phase = 0  # Reset to forward tests for next round
                    
                    # Update test sequence status
                    print("Movement test sequence completed")
                    last_movement_time = current_time
                
                # Sleep between checks
                time.sleep(1.0)
            
            # Calculate statistics
            if results['voltage_readings']:
                avg_voltage = sum(results['voltage_readings']) / len(results['voltage_readings'])
                min_voltage = min(results['voltage_readings'])
                max_voltage = max(results['voltage_readings'])
                mode_counts = {}
                for mode in results['power_modes']:
                    mode_counts[mode] = mode_counts.get(mode, 0) + 1
                
                print("\n--- Diagnostic Summary ---")
                print(f"Test duration: {duration} seconds")
                print(f"Voltage range: {min_voltage:.2f}V - {max_voltage:.2f}V (avg: {avg_voltage:.2f}V)")
                print("Power modes observed:")
                for mode, count in mode_counts.items():
                    print(f"  - {mode}: {count} times ({count/len(results['power_modes'])*100:.1f}%)")
                
                # Check if we observed any mode transitions
                if len(set(results['power_modes'])) > 1:
                    print("\nPower mode transitions observed:")
                    last_mode = results['power_modes'][0]
                    for i, mode in enumerate(results['power_modes'][1:], 1):
                        if mode != last_mode:
                            voltage = results['voltage_readings'][i]
                            timestamp = results['timestamps'][i] - start_time
                            print(f"  {last_mode} -> {mode} at {timestamp:.1f}s (Voltage: {voltage:.2f}V)")
                            last_mode = mode
                else:
                    print("\nNo power mode transitions observed during the test.")
                
                # Movement test summary if applicable
                if test_movement and results['movement_tests']:
                    print("\nMovement Test Summary:")
                    # Group by test type
                    linear_x_tests = [t for t in results['movement_tests'] if t['type'] == 'linear_x']
                    linear_y_tests = [t for t in results['movement_tests'] if t['type'] == 'linear_y'] 
                    angular_tests = [t for t in results['movement_tests'] if t['type'] == 'angular']
                    combined_tests = [t for t in results['movement_tests'] if t['type'].startswith('combined')]
                    
                    # Report on X-axis tests
                    if linear_x_tests:
                        limited_count = sum(1 for t in linear_x_tests if t['limited_by_rules'])
                        print(f"X direction speed tests: {len(linear_x_tests)} tests performed")
                        print(f"  Tests limited by rules: {limited_count}/{len(linear_x_tests)}")
                        
                        # Group by power mode
                        by_mode = {}
                        for test in linear_x_tests:
                            mode = test['power_mode']
                            if mode not in by_mode:
                                by_mode[mode] = []
                            by_mode[mode].append(test)
                        
                        # Print details for each mode
                        for mode, tests in by_mode.items():
                            # Find max absolute speed permitted
                            max_positive = max([t['actual'] for t in tests if t['actual'] >= 0], default=0)
                            max_negative = min([t['actual'] for t in tests if t['actual'] < 0], default=0)
                            print(f"  Mode {mode}: Max X speed permitted: {max_positive:.2f}m/s forward, {abs(max_negative):.2f}m/s backward")
                    
                    # Report on Y-axis tests
                    if linear_y_tests:
                        limited_count = sum(1 for t in linear_y_tests if t['limited_by_rules'])
                        print(f"Y direction speed tests: {len(linear_y_tests)} tests performed")
                        print(f"  Tests limited by rules: {limited_count}/{len(linear_y_tests)}")
                        
                        # Group by power mode
                        by_mode = {}
                        for test in linear_y_tests:
                            mode = test['power_mode']
                            if mode not in by_mode:
                                by_mode[mode] = []
                            by_mode[mode].append(test)
                        
                        # Print details for each mode
                        for mode, tests in by_mode.items():
                            # Find max absolute speed permitted
                            max_positive = max([t['actual'] for t in tests if t['actual'] >= 0], default=0)
                            max_negative = min([t['actual'] for t in tests if t['actual'] < 0], default=0)
                            print(f"  Mode {mode}: Max Y speed permitted: {max_positive:.2f}m/s right, {abs(max_negative):.2f}m/s left")
                    
                    # Report on angular tests
                    if angular_tests:
                        limited_count = sum(1 for t in angular_tests if t['limited_by_rules'])
                        print(f"Angular rotation tests: {len(angular_tests)} tests performed")
                        print(f"  Tests limited by rules: {limited_count}/{len(angular_tests)}")
                        
                        # Group by power mode
                        by_mode = {}
                        for test in angular_tests:
                            mode = test['power_mode']
                            if mode not in by_mode:
                                by_mode[mode] = []
                            by_mode[mode].append(test)
                        
                        # Print details for each mode
                        for mode, tests in by_mode.items():
                            # Find max absolute rotation permitted
                            max_positive = max([t['actual'] for t in tests if t['actual'] >= 0], default=0)
                            max_negative = min([t['actual'] for t in tests if t['actual'] < 0], default=0)
                            print(f"  Mode {mode}: Max rotation permitted: {max_positive:.2f}rad/s CCW, {abs(max_negative):.2f}rad/s CW")
            
            return results
            
        finally:
            # Clean up test movements
            if test_movement and cmd_vel_publisher:
                # Stop the robot
                stop_cmd = {
                    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
                }
                cmd_vel_publisher.publish(roslibpy.Message(stop_cmd))
                cmd_vel_publisher.unadvertise()
                print("Movement testing completed and robot stopped.")

    # Example usage
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Battery Monitor with Rule-Based System')
    parser.add_argument('--test', action='store_true', help='Run rule-based system tests')
    parser.add_argument('--simulate', action='store_true', help='Simulate voltage drop')
    parser.add_argument('--monitor', action='store_true', help='Start battery monitoring')
    parser.add_argument('--diagnose', action='store_true', help='Run live diagnostics with ROS')
    parser.add_argument('--move', action='store_true', help='Test with movement during diagnosis')
    parser.add_argument('--duration', type=int, default=60, help='Duration for diagnostics (seconds)')
    parser.add_argument('--speed-levels', type=int, default=3, help='Number of speed levels to test per direction')
    parser.add_argument('--rotation-levels', type=int, default=3, help='Number of rotation levels to test per direction')
    parser.add_argument('--max-speed', type=float, default=0.6, help='Maximum speed to test (m/s)')
    parser.add_argument('--max-rotation', type=float, default=2.0, help='Maximum rotation to test (rad/s)')
    parser.add_argument('--omnidirectional', action='store_true', help='Test sideways movement (Y axis)')
    parser.add_argument('--bidirectional', action='store_true', help='Test negative speeds and rotations')
    
    args = parser.parse_args()
    
    # Create battery monitor instance
    monitor = BatteryMonitor()
    
    if args.test:
        # Test rule-based system without connecting to ROS
        print("\nTesting rule-based system...")
        monitor.test_rule_based_system(base_speed=args.max_speed)
    
    if args.simulate:
        # Simulate voltage drop without connecting to ROS
        print("\nSimulating voltage drop...")
        monitor.simulate_voltage_drop()
    
    if args.diagnose:
        # Run live diagnostics (connects to ROS)
        monitor.run_live_diagnostics(
            duration=args.duration,
            test_movement=args.move,
            speed_levels=args.speed_levels,
            rotation_levels=args.rotation_levels,
            max_speed=args.max_speed,
            max_rotation=args.max_rotation,
            test_omnidirectional=args.omnidirectional,
            test_bidirectional=args.bidirectional
        )
    
    if args.monitor:
        # Start actual monitoring (requires ROS connection)
        if monitor.connect():
            try:
                monitor.run()
            except Exception as e:
                print(f"Error: {e}")
            finally:
                monitor.cleanup()
    
    if not any([args.test, args.simulate, args.monitor, args.diagnose]):
        # If no arguments provided, run tests and show usage
        print("Running tests by default...")
        monitor.test_rule_based_system(base_speed=0.5)
        
        print("\nFor battery monitoring, use: python battery_monitor.py --monitor")
        print("For testing rule-based system: python battery_monitor.py --test")
        print("For simulating voltage drop: python battery_monitor.py --simulate")
        print("For MentorPi M1 diagnostics: python battery_monitor.py --diagnose --move --omnidirectional --bidirectional")
        print("  With custom settings: --speed-levels 3 --rotation-levels 3 --max-speed 0.6 --max-rotation 2.0")