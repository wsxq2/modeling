#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess
import threading
import signal
import os
from datetime import datetime


class BagRecorderControl(Node):
    """
    ROS2 node to control bag recording with PS3 joystick X button.
    First X press: start recording /imu and /cx/pointcloud topics
    Second X press: stop recording (send Ctrl+C to subprocess)
    """
    
    def __init__(self):
        super().__init__('bag_recorder_control')
        
        # Parameters
        self.declare_parameter('x_button_index', 2)  # X button index
        self.declare_parameter('topics', ['/imu', '/cx/pointcloud'])
        
        self.x_button_index = self.get_parameter('x_button_index').value
        self.topics = self.get_parameter('topics').value
        
        # State variables
        self.is_recording = False
        self.previous_button_state = 0
        self.record_process = None
        
        # LED control paths
        self.led_trigger_path = '/sys/class/leds/led0/trigger'
        self.led_brightness_path = '/sys/class/leds/led0/brightness'
        
        # Initialize LED for manual control
        self.setup_led()
        
        # Subscribe to joy topic
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        self.get_logger().info(f'Bag recorder control node started')
        self.get_logger().info(f'X button index: {self.x_button_index}')
        self.get_logger().info(f'Topics to record: {self.topics}')
        self.get_logger().info('LED0 initialized for recording status indication')
        
    def joy_callback(self, msg):
        """Handle joystick messages and detect X button press"""
        if len(msg.buttons) <= self.x_button_index:
            return
            
        current_button_state = msg.buttons[self.x_button_index]
        
        # Detect button press (transition from 0 to 1)
        if current_button_state == 1 and self.previous_button_state == 0:
            self.handle_x_button_press()
            
        self.previous_button_state = current_button_state
        
    def setup_led(self):
        """Initialize LED0 for manual control"""
        try:
            # Set LED0 to manual control mode
            with open(self.led_trigger_path, 'w') as f:
                f.write('none')
            # Turn off LED initially
            with open(self.led_brightness_path, 'w') as f:
                f.write('0')
            self.get_logger().info('LED0 setup completed')
        except Exception as e:
            self.get_logger().warning(f'Failed to setup LED0: {str(e)}')
            
    def set_led_state(self, state):
        """Set LED0 state (True=on, False=off)"""
        try:
            brightness = '1' if state else '0'
            with open(self.led_brightness_path, 'w') as f:
                f.write(brightness)
        except Exception as e:
            self.get_logger().warning(f'Failed to set LED0 state: {str(e)}')
            
    def handle_x_button_press(self):
        """Handle X button press - toggle recording"""
        if not self.is_recording:
            self.start_recording()
        else:
            self.stop_recording()
            
    def start_recording(self):
        """Start bag recording"""
        try:
            # Generate timestamped bag filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = f"recorded_data_{timestamp}"
            
            # Build ros2 bag record command
            cmd = ['ros2', 'bag', 'record'] + self.topics + ['-o', bag_name]
            
            self.get_logger().info(f'Starting bag recording: {" ".join(cmd)}')
            
            # Start subprocess
            self.record_process = subprocess.Popen(
                cmd,
                cwd=os.path.expanduser('~'),  # Record in home directory
                preexec_fn=os.setsid  # Create new process group for proper termination
            )
            
            self.is_recording = True
            
            # Turn on LED0 to indicate recording
            self.set_led_state(True)
            
            self.get_logger().info(f'Bag recording started with PID: {self.record_process.pid}')
            self.get_logger().info('LED0 turned ON - Recording active')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {str(e)}')
            
    def stop_recording(self):
        """Stop bag recording by sending SIGINT to the process group"""
        if self.record_process and self.is_recording:
            try:
                # Send SIGINT to the entire process group (like Ctrl+C)
                os.killpg(os.getpgid(self.record_process.pid), signal.SIGINT)
                
                self.get_logger().info('Stopping bag recording (SIGINT sent)')
                
                # Wait for process to terminate
                self.record_process.wait(timeout=5.0)
                
                self.record_process = None
                self.is_recording = False
                
                # Turn off LED0 to indicate recording stopped
                self.set_led_state(False)
                
                self.get_logger().info('Bag recording stopped successfully')
                self.get_logger().info('LED0 turned OFF - Recording stopped')
                
            except subprocess.TimeoutExpired:
                self.get_logger().warning('Process did not terminate gracefully, force killing...')
                os.killpg(os.getpgid(self.record_process.pid), signal.SIGKILL)
                self.record_process = None
                self.is_recording = False
                
                # Turn off LED0 even if force killed
                self.set_led_state(False)
                
            except Exception as e:
                self.get_logger().error(f'Failed to stop recording: {str(e)}')
                
    def __del__(self):
        """Cleanup when node is destroyed"""
        if self.is_recording and self.record_process:
            self.stop_recording()
        # Ensure LED is turned off when node is destroyed
        self.set_led_state(False)


def main(args=None):
    rclpy.init(args=args)
    
    node = BagRecorderControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure recording is stopped before shutting down
        if node.is_recording:
            node.stop_recording()
        # Turn off LED when shutting down
        node.set_led_state(False)
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == '__main__':
    main()