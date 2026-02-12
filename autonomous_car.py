#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Autonomous RC Car - Main Control Program
Implements obstacle avoidance using ultrasonic sensors and optional camera
"""
import time
import sys
import signal
from typing import Optional
import config
from ultrasonic_sensors import SensorArray
from motor_control import MotorController, Direction

# Camera is optional
try:
    from camera_module import CameraModule, CV2_AVAILABLE
    CAMERA_MODULE_AVAILABLE = True
except ImportError as e:
    CAMERA_MODULE_AVAILABLE = False
    CV2_AVAILABLE = False
    print(f"Warning: Camera module unavailable: {e}")


class AutonomousCar:
    """Main controller for autonomous RC car"""
    
    def __init__(self, enable_camera: bool = False):
        print("Initializing Autonomous RC Car...")
        
        # Initialize components
        self.sensors = SensorArray()
        self.motors = MotorController()
        
        # Only enable camera if module is available and has opencv
        if enable_camera and CAMERA_MODULE_AVAILABLE and CV2_AVAILABLE:
            try:
                self.camera = CameraModule(use_picamera=True)
                print("Camera enabled")
            except Exception as e:
                print(f"Warning: Could not initialize camera: {e}")
                self.camera = None
                enable_camera = False
        else:
            self.camera = None
            if enable_camera:
                print("Warning: Camera requested but opencv-python not installed")
                print("Install with: pip install opencv-python")
                enable_camera = False
        
        self.running = False
        self.enable_camera = enable_camera
        
        # Statistics
        self.distance_traveled = 0
        self.obstacles_avoided = 0
        self.start_time = None
        
        print("Initialization complete!\n")
    
    def analyze_sensors(self) -> dict:
        """
        Analyze sensor data and determine best action
        Returns dict with distances and recommended action
        """
        distances = self.sensors.get_safe_distances()
        
        left_dist = distances['left']
        center_dist = distances['center']
        right_dist = distances['right']
        
        # Determine situation
        analysis = {
            'distances': distances,
            'action': None,
            'speed': config.SPEED_NORMAL,
            'severity': 'safe'
        }
        
        # Critical danger - immediate stop
        if center_dist < config.CRITICAL_DISTANCE:
            analysis['action'] = 'emergency_stop'
            analysis['severity'] = 'critical'
            return analysis
        
        # Obstacle ahead - need to navigate
        if center_dist < config.MIN_DISTANCE:
            analysis['obstacles'] = True
            analysis['severity'] = 'warning'
            
            # Choose direction based on which side has more space
            if left_dist > right_dist and left_dist > config.MIN_DISTANCE:
                analysis['action'] = 'turn_left'
            elif right_dist > left_dist and right_dist > config.MIN_DISTANCE:
                analysis['action'] = 'turn_right'
            elif left_dist > config.MIN_DISTANCE:
                analysis['action'] = 'turn_left'
            elif right_dist > config.MIN_DISTANCE:
                analysis['action'] = 'turn_right'
            else:
                # No good option, reverse
                analysis['action'] = 'reverse'
        
        # Path mostly clear, but adjust for side obstacles
        elif center_dist >= config.MIN_DISTANCE:
            # Check side sensors for gentle corrections
            if left_dist < config.MIN_DISTANCE and right_dist > config.SAFE_DISTANCE:
                analysis['action'] = 'gentle_right'
                analysis['severity'] = 'caution'
            elif right_dist < config.MIN_DISTANCE and left_dist > config.SAFE_DISTANCE:
                analysis['action'] = 'gentle_left'
                analysis['severity'] = 'caution'
            else:
                # All clear, move forward
                analysis['action'] = 'forward'
                analysis['severity'] = 'safe'
        
        return analysis
    
    def execute_action(self, analysis: dict):
        """Execute the recommended action"""
        action = analysis['action']
        
        if action == 'emergency_stop':
            print("🛑 EMERGENCY STOP!")
            self.motors.stop()
            time.sleep(0.5)
            # Back up
            self.motors.backward(speed=config.SPEED_SLOW)
            time.sleep(0.5)
            self.motors.stop()
            self.obstacles_avoided += 1
            
        elif action == 'reverse':
            print("⬅️ Reversing...")
            self.motors.backward(speed=config.SPEED_SLOW)
            time.sleep(0.8)
            self.motors.stop()
            time.sleep(0.2)
            # Then turn
            if analysis['distances']['left'] > analysis['distances']['right']:
                self.motors.turn_left()
                time.sleep(config.TURN_DURATION)
            else:
                self.motors.turn_right()
                time.sleep(config.TURN_DURATION)
            self.motors.stop()
            self.obstacles_avoided += 1
            
        elif action == 'turn_left':
            print("↰ Turning left...")
            self.motors.turn_left()
            time.sleep(config.TURN_DURATION)
            self.motors.stop()
            self.obstacles_avoided += 1
            
        elif action == 'turn_right':
            print("↱ Turning right...")
            self.motors.turn_right()
            time.sleep(config.TURN_DURATION)
            self.motors.stop()
            self.obstacles_avoided += 1
            
        elif action == 'gentle_left':
            print("↖️ Gentle left...")
            self.motors.gentle_left(speed=config.SPEED_NORMAL)
            
        elif action == 'gentle_right':
            print("↗️ Gentle right...")
            self.motors.gentle_right(speed=config.SPEED_NORMAL)
            
        elif action == 'forward':
            # Adjust speed based on distances
            min_distance = min(analysis['distances'].values())
            if min_distance < config.SAFE_DISTANCE:
                speed = config.SPEED_SLOW
            else:
                speed = config.SPEED_NORMAL
            
            self.motors.forward(speed=speed)
    
    def print_status(self, analysis: dict):
        """Print current status to console"""
        distances = analysis['distances']
        severity_icons = {
            'safe': '✅',
            'caution': '⚠️',
            'warning': '⚠️',
            'critical': '🛑'
        }
        
        icon = severity_icons.get(analysis['severity'], '❓')
        
        # Clear line and print status
        status = (f"\r{icon} L:{distances['left']:>5.1f}cm | "
                 f"C:{distances['center']:>5.1f}cm | "
                 f"R:{distances['right']:>5.1f}cm | "
                 f"Action: {analysis['action']:>15} | "
                 f"Obstacles avoided: {self.obstacles_avoided}")
        
        print(status, end='', flush=True)
    
    def run(self):
        """Main autonomous driving loop"""
        print("🚗 Starting autonomous mode...")
        print("Press Ctrl+C to stop\n")
        
        self.running = True
        self.start_time = time.time()
        
        try:
            while self.running:
                # Read sensors
                analysis = self.analyze_sensors()
                
                # Execute action
                self.execute_action(analysis)
                
                # Print status
                self.print_status(analysis)
                
                # Optional: Capture camera frame
                if self.enable_camera and self.camera:
                    frame = self.camera.capture_frame()
                    # Could add vision-based obstacle detection here
                
                # Wait before next scan
                time.sleep(config.SCAN_INTERVAL)
                
        except KeyboardInterrupt:
            print("\n\n⏸️ Stopping...")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown of all systems"""
        print("\n🔧 Shutting down systems...")
        
        self.running = False
        
        # Stop motors
        self.motors.stop()
        
        # Calculate runtime
        if self.start_time:
            runtime = time.time() - self.start_time
            print(f"\n📊 Session Statistics:")
            print(f"   Runtime: {runtime:.1f} seconds")
            print(f"   Obstacles avoided: {self.obstacles_avoided}")
        
        # Cleanup
        print("\n🧹 Cleaning up...")
        self.motors.cleanup()
        self.sensors.cleanup()
        if self.camera:
            self.camera.cleanup()
        
        print("✅ Shutdown complete!")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\n⚠️ Interrupt received, shutting down...")
    sys.exit(0)


if __name__ == "__main__":
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Parse command line arguments
    enable_camera = '--camera' in sys.argv or '-c' in sys.argv
    
    print("=" * 60)
    print("🚗 AUTONOMOUS RC CAR")
    print("=" * 60)
    print(f"Camera: {'Enabled' if enable_camera else 'Disabled'}")
    print(f"Sensors: 3 x Ultrasonic (Left, Center, Right)")
    print("=" * 60)
    print()
    
    # Create and run car
    car = AutonomousCar(enable_camera=enable_camera)
    car.run()
