"""
Squirrel Detection and Chase Mode
Autonomous RC car that detects and chases squirrels using PIR and camera
"""
import time
import sys
import signal
from typing import Optional
import config
from ultrasonic_sensors import SensorArray
from motor_control import MotorController, Direction
from camera_module import CameraModule
from motion_detection import MotionDetectionManager, MotionEvent


class SquirrelChaserCar:
    """RC Car that detects and chases squirrels"""
    
    def __init__(self, enable_camera: bool = True, enable_pir: bool = True):
        print("Initializing Squirrel Chaser Car...")
        
        # Initialize components
        self.sensors = SensorArray()
        self.motors = MotorController()
        self.camera = CameraModule(use_picamera=True) if enable_camera else None
        self.motion_detector = MotionDetectionManager(enable_pir=enable_pir, enable_camera=enable_camera)
        
        self.running = False
        self.enable_camera = enable_camera
        self.enable_pir = enable_pir
        
        # Chase state
        self.in_chase = False
        self.chase_start_time = None
        self.chase_direction = None
        
        # Statistics
        self.squirrels_detected = 0
        self.chases_completed = 0
        self.start_time = None
        
        print("🐿️ Squirrel Chaser ready!\n")
    
    def detect_squirrel(self) -> Optional[MotionEvent]:
        """
        Check for squirrel/motion using PIR and camera
        Returns MotionEvent if detected, None otherwise
        """
        # Check PIR sensor first (faster)
        motion_event = self.motion_detector.check_motion()
        
        if motion_event:
            print(f"\n🔍 PIR detected motion!")
            
            # If camera is enabled, verify with camera
            if self.camera and self.enable_camera:
                frame = self.camera.capture_frame()
                if frame is not None:
                    detected, center, area, debug_frame = self.camera.detect_motion(frame)
                    
                    if detected:
                        direction = self.camera.get_motion_direction(center)
                        motion_event.position = center
                        motion_event.area = area
                        print(f"📷 Camera confirmed motion in {direction} quadrant (area: {area})")
                        return motion_event
                    else:
                        print("📷 Camera did not confirm motion - false alarm")
                        return None
            
            return motion_event
        
        # If no PIR, try camera-only detection
        elif self.camera and self.enable_camera:
            frame = self.camera.capture_frame()
            if frame is not None:
                detected, center, area, debug_frame = self.camera.detect_motion(frame)
                
                if detected and area > config.MOTION_AREA_THRESHOLD:
                    direction = self.camera.get_motion_direction(center)
                    motion_event = MotionEvent(time.time(), source="Camera")
                    motion_event.position = center
                    motion_event.area = area
                    print(f"\n📷 Camera detected motion in {direction} quadrant (area: {area})")
                    return motion_event
        
        return None
    
    def chase_squirrel(self, motion_event: MotionEvent):
        """Execute chase sequence toward detected motion"""
        self.in_chase = True
        self.chase_start_time = time.time()
        self.squirrels_detected += 1
        
        print(f"\n🏃 CHASE #{self.squirrels_detected} INITIATED!")
        
        # Determine chase direction
        chase_direction = 'forward'
        
        if motion_event.position and self.camera:
            camera_direction = self.camera.get_motion_direction(motion_event.position)
            
            if camera_direction == 'left':
                print("   Squirrel is to the LEFT!")
                self.motors.turn_left(speed=config.CHASE_SPEED)
                time.sleep(0.3)
                chase_direction = 'left'
            elif camera_direction == 'right':
                print("   Squirrel is to the RIGHT!")
                self.motors.turn_right(speed=config.CHASE_SPEED)
                time.sleep(0.3)
                chase_direction = 'right'
        
        # Chase forward with obstacle avoidance
        chase_end_time = time.time() + config.CHASE_DURATION
        
        while time.time() < chase_end_time and self.running:
            # Check for obstacles while chasing
            distances = self.sensors.get_safe_distances()
            
            if distances['center'] < config.CRITICAL_DISTANCE:
                print("   ⚠️ Obstacle detected - stopping chase!")
                self.motors.stop()
                break
            
            # Adjust speed based on obstacles
            if distances['center'] < config.MIN_DISTANCE:
                speed = config.SPEED_SLOW
            else:
                speed = config.CHASE_SPEED
            
            self.motors.forward(speed=speed)
            
            # Display chase progress
            elapsed = time.time() - self.chase_start_time
            remaining = config.CHASE_DURATION - elapsed
            print(f"\r   Chasing... {remaining:.1f}s remaining", end='', flush=True)
            
            time.sleep(0.1)
        
        # End chase
        self.motors.stop()
        self.chases_completed += 1
        print(f"\n✅ Chase complete! (Total: {self.chases_completed})")
        
        # Cooldown
        print(f"   Cooling down for {config.MOTION_COOLDOWN}s...")
        self.motion_detector.trigger_cooldown()
        self.camera.reset_motion_detection() if self.camera else None
        
        self.in_chase = False
    
    def patrol_mode(self):
        """
        Patrol mode - slowly move around looking for squirrels
        """
        distances = self.sensors.get_safe_distances()
        
        # Simple patrol logic
        if distances['center'] > config.SAFE_DISTANCE:
            self.motors.forward(speed=config.SPEED_SLOW)
        elif distances['left'] > distances['right']:
            self.motors.gentle_left(speed=config.SPEED_SLOW)
        else:
            self.motors.gentle_right(speed=config.SPEED_SLOW)
    
    def run(self):
        """Main squirrel detection loop"""
        print("=" * 60)
        print("🐿️ SQUIRREL CHASER MODE ACTIVE")
        print("=" * 60)
        print(f"PIR Sensor: {'Enabled' if self.enable_pir else 'Disabled'}")
        print(f"Camera: {'Enabled' if self.enable_camera else 'Disabled'}")
        print("Press Ctrl+C to stop\n")
        print("Waiting for squirrels...\n")
        
        self.running = True
        self.start_time = time.time()
        
        try:
            while self.running:
                # Check for squirrel detection
                motion_event = self.detect_squirrel()
                
                if motion_event and not self.in_chase:
                    # Squirrel detected - chase it!
                    self.chase_squirrel(motion_event)
                
                elif not self.in_chase:
                    # No squirrel - patrol mode
                    self.patrol_mode()
                    
                    # Status update
                    print(f"\r🔍 Scanning... (Detected: {self.squirrels_detected}, Chased: {self.chases_completed})", 
                          end='', flush=True)
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n\n⏸️ Stopping squirrel chaser...")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown of all systems"""
        print("\n\n🔧 Shutting down systems...")
        
        self.running = False
        
        # Stop motors
        self.motors.stop()
        
        # Calculate runtime
        if self.start_time:
            runtime = time.time() - self.start_time
            print(f"\n📊 Session Statistics:")
            print(f"   Runtime: {runtime:.1f} seconds ({runtime/60:.1f} minutes)")
            print(f"   Squirrels detected: {self.squirrels_detected}")
            print(f"   Chases completed: {self.chases_completed}")
            if self.squirrels_detected > 0:
                print(f"   Success rate: {(self.chases_completed/self.squirrels_detected*100):.1f}%")
        
        # Cleanup
        print("\n🧹 Cleaning up...")
        self.motors.cleanup()
        self.sensors.cleanup()
        self.motion_detector.cleanup()
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
    enable_camera = '--camera' in sys.argv or '-c' in sys.argv or '--no-pir' not in sys.argv
    enable_pir = '--pir' in sys.argv or '-p' in sys.argv or '--no-pir' not in sys.argv
    
    if '--no-camera' in sys.argv:
        enable_camera = False
    if '--no-pir' in sys.argv:
        enable_pir = False
    
    print("=" * 60)
    print("🐿️ SQUIRREL CHASER RC CAR")
    print("=" * 60)
    print(f"PIR Motion Sensor: {'Enabled' if enable_pir else 'Disabled'}")
    print(f"Camera Motion Detection: {'Enabled' if enable_camera else 'Disabled'}")
    print("=" * 60)
    print()
    
    # Create and run car
    car = SquirrelChaserCar(enable_camera=enable_camera, enable_pir=enable_pir)
    car.run()
