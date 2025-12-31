"""
Motion Detection Module
Handles PIR sensor for detecting squirrels and other moving objects
"""
import RPi.GPIO as GPIO
import time
from typing import Optional, Callable
import threading
import config


class PIRMotionSensor:
    """PIR (Passive Infrared) Motion Sensor for detecting movement"""
    
    def __init__(self, pin: int = None, callback: Callable = None):
        """
        Initialize PIR sensor
        Args:
            pin: GPIO pin number for PIR sensor
            callback: Function to call when motion is detected
        """
        self.pin = pin or config.PIR_SENSOR_PIN
        self.callback = callback
        self.last_detection_time = 0
        self.motion_detected = False
        self.enabled = config.PIR_ENABLE
        
        if self.enabled:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.IN)
            
            # Allow sensor to stabilize (PIR sensors need ~30 seconds on first power-up)
            print("PIR sensor initializing (waiting 2 seconds for stabilization)...")
            time.sleep(2)
            print("PIR sensor ready!")
    
    def is_motion_detected(self) -> bool:
        """
        Check if motion is currently detected
        Returns True if motion is detected, False otherwise
        """
        if not self.enabled:
            return False
        
        current_time = time.time()
        
        # Debounce - ignore rapid triggers
        if current_time - self.last_detection_time < config.PIR_DEBOUNCE_TIME:
            return False
        
        motion = GPIO.input(self.pin) == GPIO.HIGH
        
        if motion:
            self.last_detection_time = current_time
            self.motion_detected = True
            
            if self.callback:
                self.callback()
        else:
            self.motion_detected = False
        
        return motion
    
    def wait_for_motion(self, timeout: Optional[float] = None) -> bool:
        """
        Block until motion is detected or timeout
        Args:
            timeout: Maximum seconds to wait (None = wait forever)
        Returns:
            True if motion detected, False if timeout
        """
        if not self.enabled:
            return False
        
        start_time = time.time()
        
        while True:
            if self.is_motion_detected():
                return True
            
            if timeout and (time.time() - start_time > timeout):
                return False
            
            time.sleep(0.1)
    
    def start_monitoring(self, callback: Callable):
        """
        Start continuous monitoring in background thread
        Args:
            callback: Function to call when motion detected
        """
        if not self.enabled:
            return
        
        self.callback = callback
        self.monitoring = True
        
        def monitor():
            while self.monitoring:
                if self.is_motion_detected() and self.callback:
                    self.callback()
                time.sleep(0.1)
        
        self.monitor_thread = threading.Thread(target=monitor, daemon=True)
        self.monitor_thread.start()
    
    def stop_monitoring(self):
        """Stop background monitoring"""
        self.monitoring = False
    
    def cleanup(self):
        """Clean up GPIO"""
        if self.enabled:
            GPIO.cleanup(self.pin)


class MotionEvent:
    """Represents a motion detection event"""
    
    def __init__(self, timestamp: float, source: str = "PIR"):
        self.timestamp = timestamp
        self.source = source  # "PIR" or "Camera"
        self.position = None  # Will be set by camera-based detection
        self.area = None  # Size of detected motion
    
    def __repr__(self):
        return f"MotionEvent(source={self.source}, time={self.timestamp:.2f})"


class MotionDetectionManager:
    """Manages multiple motion detection sources"""
    
    def __init__(self, enable_pir: bool = True, enable_camera: bool = True):
        self.enable_pir = enable_pir and config.PIR_ENABLE
        self.enable_camera = enable_camera
        
        self.pir_sensor = None
        self.last_motion_event = None
        self.motion_callbacks = []
        self.in_cooldown = False
        self.cooldown_until = 0
        
        # Initialize PIR if enabled
        if self.enable_pir:
            try:
                self.pir_sensor = PIRMotionSensor()
            except Exception as e:
                print(f"Warning: Could not initialize PIR sensor: {e}")
                self.pir_sensor = None
    
    def check_motion(self) -> Optional[MotionEvent]:
        """
        Check all motion detection sources
        Returns MotionEvent if motion detected, None otherwise
        """
        current_time = time.time()
        
        # Check if in cooldown period
        if self.in_cooldown and current_time < self.cooldown_until:
            return None
        else:
            self.in_cooldown = False
        
        # Check PIR sensor
        if self.pir_sensor and self.pir_sensor.is_motion_detected():
            event = MotionEvent(current_time, source="PIR")
            self.last_motion_event = event
            return event
        
        return None
    
    def trigger_cooldown(self):
        """Start cooldown period after chase"""
        self.in_cooldown = True
        self.cooldown_until = time.time() + config.MOTION_COOLDOWN
    
    def add_callback(self, callback: Callable):
        """Add callback function for motion detection"""
        self.motion_callbacks.append(callback)
    
    def notify_motion(self, event: MotionEvent):
        """Notify all callbacks of motion detection"""
        for callback in self.motion_callbacks:
            try:
                callback(event)
            except Exception as e:
                print(f"Error in motion callback: {e}")
    
    def cleanup(self):
        """Clean up all sensors"""
        if self.pir_sensor:
            self.pir_sensor.cleanup()


if __name__ == "__main__":
    """Test motion detection"""
    print("Testing Motion Detection System...")
    print("Wave your hand in front of the PIR sensor")
    print("Press Ctrl+C to exit\n")
    
    def on_motion_detected():
        print(f"🐿️ MOTION DETECTED at {time.time():.2f}")
    
    manager = MotionDetectionManager(enable_pir=True, enable_camera=False)
    manager.add_callback(on_motion_detected)
    
    try:
        print("Monitoring for motion...")
        while True:
            event = manager.check_motion()
            if event:
                print(f"Event: {event}")
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print("\n\nCleaning up...")
        manager.cleanup()
        print("Done!")
