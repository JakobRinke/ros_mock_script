import roslibpy
import time

class SimpleBatteryMonitor:
    def __init__(self, host='192.168.149.1', port=9091):
        self.host = host
        self.port = port
        self.client = None
        self.battery_sub = None
        self.power_mode_pub = None
        
        # Voltage thresholds for 2S LiPo
        self.MIN_VOLTAGE = 6.0  # absolute minimum
        self.CRITICAL_VOLTAGE = 6.6  # 3.3V/cell
        self.CONSERVATIVE_VOLTAGE = 7.0
        self.EFFICIENT_VOLTAGE = 7.6
        self.MAX_VOLTAGE = 8.4  # fully charged (4.2V/cell)
        
        # Battery status storage
        self.current_voltage = None
        self.current_percentage = None
        self.current_mode = "NORMAL"
        
    def connect(self):
        print(f"Connecting to ROS bridge at {self.host}:{self.port}...")
        self.client = roslibpy.Ros(host=self.host, port=self.port)
        self.client.run()
        
        if self.client.is_connected:
            print("Connected to ROS bridge.")
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
            
            # Initialize battery status
            self.initialize_battery_status()
            return True
        else:
            print("Failed to connect to ROS bridge.")
            return False
            
    def initialize_battery_status(self):
        """Request and display initial battery status"""
        print("Initializing battery status...")
        # Wait briefly for first message
        start_time = time.time()
        timeout = 5.0  # seconds
        while self.current_voltage is None and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        if self.current_voltage is not None:
            print(f"Initial Battery Status - Voltage: {self.current_voltage:.2f} V, "
                  f"Battery: {self.current_percentage:.1f}%, Mode: {self.current_mode}")
        else:
            print("Warning: Could not retrieve initial battery status within timeout")
    
    def battery_callback(self, message):
        # Convert from millivolts to volts
        voltage_mv = message['data']
        voltage = voltage_mv / 1000.0
        
        # Update stored status
        self.current_voltage = voltage
        self.current_percentage = self.convert_voltage_to_percentage(voltage)
        new_mode = self.determine_power_mode(voltage)
        
        print(f"Voltage: {self.current_voltage:.2f} V, Battery: {self.current_percentage:.1f}%, Mode: {new_mode}")
        
        if new_mode != self.current_mode:
            self.current_mode = new_mode
            if self.power_mode_pub:
                self.power_mode_pub.publish(roslibpy.Message({'data': new_mode}))
                print(f"Power mode changed to: {new_mode}")
    
    def convert_voltage_to_percentage(self, voltage):
        # Clamp voltage to realistic LiPo range
        clamped_voltage = max(self.MIN_VOLTAGE, min(self.MAX_VOLTAGE, voltage))
        percentage = ((clamped_voltage - self.MIN_VOLTAGE) / (self.MAX_VOLTAGE - self.MIN_VOLTAGE)) * 100.0
        return round(percentage, 1)
    
    def determine_power_mode(self, voltage):
        if voltage >= self.EFFICIENT_VOLTAGE:
            return "NORMAL"
        elif voltage >= self.CONSERVATIVE_VOLTAGE:
            return "EFFICIENT"
        elif voltage >= self.CRITICAL_VOLTAGE:
            return "CONSERVATIVE"
        else:
            return "CRITICAL"
    
    def list_topics(self):
        if self.client and self.client.is_connected:
            topics = self.client.get_topics()
            print("Available topics:", topics)
            return topics
        else:
            print("Not connected to ROS bridge.")
            return []
    
    def run(self):
        try:
            print("Battery Monitor running. Press Ctrl+C to exit.")
            while self.client and self.client.is_connected:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Battery Monitor stopped by user.")
        finally:
            self.cleanup()
    
    def cleanup(self):
        print("Cleaning up resources...")
        if self.battery_sub:
            self.battery_sub.unsubscribe()
        if self.power_mode_pub:
            self.power_mode_pub.unadvertise()
        if self.client and self.client.is_connected:
            self.client.terminate()

if __name__ == "__main__":
    monitor = SimpleBatteryMonitor()
    if monitor.connect():
        try:
            monitor.run()
        except Exception as e:
            print(f"Error: {e}")
            monitor.cleanup()