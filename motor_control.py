"""
Motor Control Module
Handles motor control for RC car movement
Supports L298N or similar H-bridge motor drivers
"""
import RPi.GPIO as GPIO
import time
from enum import Enum
import config


class Direction(Enum):
    """Movement directions"""
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"


class MotorController:
    """Controls the motors for the RC car"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup motor pins
        self.setup_pins()
        
        # Setup PWM for speed control
        self.pwm_left = GPIO.PWM(config.MOTOR_LEFT_ENABLE, 1000)  # 1kHz frequency
        self.pwm_right = GPIO.PWM(config.MOTOR_RIGHT_ENABLE, 1000)
        
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        
        self.current_speed = config.SPEED_NORMAL
        self.is_moving = False
        
    def setup_pins(self):
        """Setup GPIO pins for motors"""
        pins = [
            config.MOTOR_LEFT_FORWARD,
            config.MOTOR_LEFT_BACKWARD,
            config.MOTOR_LEFT_ENABLE,
            config.MOTOR_RIGHT_FORWARD,
            config.MOTOR_RIGHT_BACKWARD,
            config.MOTOR_RIGHT_ENABLE
        ]
        
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)
    
    def set_speed(self, speed: int):
        """Set motor speed (0-100)"""
        self.current_speed = max(0, min(100, speed))
    
    def stop(self):
        """Stop all motors"""
        GPIO.output(config.MOTOR_LEFT_FORWARD, False)
        GPIO.output(config.MOTOR_LEFT_BACKWARD, False)
        GPIO.output(config.MOTOR_RIGHT_FORWARD, False)
        GPIO.output(config.MOTOR_RIGHT_BACKWARD, False)
        
        self.pwm_left.ChangeDutyCycle(0)
        self.pwm_right.ChangeDutyCycle(0)
        self.is_moving = False
    
    def forward(self, speed: int = None):
        """Move forward"""
        if speed is None:
            speed = self.current_speed
            
        GPIO.output(config.MOTOR_LEFT_FORWARD, True)
        GPIO.output(config.MOTOR_LEFT_BACKWARD, False)
        GPIO.output(config.MOTOR_RIGHT_FORWARD, True)
        GPIO.output(config.MOTOR_RIGHT_BACKWARD, False)
        
        self.pwm_left.ChangeDutyCycle(speed)
        self.pwm_right.ChangeDutyCycle(speed)
        self.is_moving = True
    
    def backward(self, speed: int = None):
        """Move backward"""
        if speed is None:
            speed = self.current_speed
            
        GPIO.output(config.MOTOR_LEFT_FORWARD, False)
        GPIO.output(config.MOTOR_LEFT_BACKWARD, True)
        GPIO.output(config.MOTOR_RIGHT_FORWARD, False)
        GPIO.output(config.MOTOR_RIGHT_BACKWARD, True)
        
        self.pwm_left.ChangeDutyCycle(speed)
        self.pwm_right.ChangeDutyCycle(speed)
        self.is_moving = True
    
    def turn_left(self, speed: int = None):
        """Turn left (left motor backward, right motor forward)"""
        if speed is None:
            speed = config.SPEED_TURN
            
        GPIO.output(config.MOTOR_LEFT_FORWARD, False)
        GPIO.output(config.MOTOR_LEFT_BACKWARD, True)
        GPIO.output(config.MOTOR_RIGHT_FORWARD, True)
        GPIO.output(config.MOTOR_RIGHT_BACKWARD, False)
        
        self.pwm_left.ChangeDutyCycle(speed)
        self.pwm_right.ChangeDutyCycle(speed)
        self.is_moving = True
    
    def turn_right(self, speed: int = None):
        """Turn right (left motor forward, right motor backward)"""
        if speed is None:
            speed = config.SPEED_TURN
            
        GPIO.output(config.MOTOR_LEFT_FORWARD, True)
        GPIO.output(config.MOTOR_LEFT_BACKWARD, False)
        GPIO.output(config.MOTOR_RIGHT_FORWARD, False)
        GPIO.output(config.MOTOR_RIGHT_BACKWARD, True)
        
        self.pwm_left.ChangeDutyCycle(speed)
        self.pwm_right.ChangeDutyCycle(speed)
        self.is_moving = True
    
    def gentle_left(self, speed: int = None):
        """Gentle left turn (slow down left motor)"""
        if speed is None:
            speed = self.current_speed
            
        GPIO.output(config.MOTOR_LEFT_FORWARD, True)
        GPIO.output(config.MOTOR_LEFT_BACKWARD, False)
        GPIO.output(config.MOTOR_RIGHT_FORWARD, True)
        GPIO.output(config.MOTOR_RIGHT_BACKWARD, False)
        
        self.pwm_left.ChangeDutyCycle(speed * 0.5)  # Left motor at 50%
        self.pwm_right.ChangeDutyCycle(speed)
        self.is_moving = True
    
    def gentle_right(self, speed: int = None):
        """Gentle right turn (slow down right motor)"""
        if speed is None:
            speed = self.current_speed
            
        GPIO.output(config.MOTOR_LEFT_FORWARD, True)
        GPIO.output(config.MOTOR_LEFT_BACKWARD, False)
        GPIO.output(config.MOTOR_RIGHT_FORWARD, True)
        GPIO.output(config.MOTOR_RIGHT_BACKWARD, False)
        
        self.pwm_left.ChangeDutyCycle(speed)
        self.pwm_right.ChangeDutyCycle(speed * 0.5)  # Right motor at 50%
        self.is_moving = True
    
    def move(self, direction: Direction, duration: float = None, speed: int = None):
        """
        Move in a specific direction for a duration
        If duration is None, movement continues until stopped
        """
        if direction == Direction.FORWARD:
            self.forward(speed)
        elif direction == Direction.BACKWARD:
            self.backward(speed)
        elif direction == Direction.LEFT:
            self.turn_left(speed)
        elif direction == Direction.RIGHT:
            self.turn_right(speed)
        elif direction == Direction.STOP:
            self.stop()
        
        if duration is not None:
            time.sleep(duration)
            self.stop()
    
    def cleanup(self):
        """Stop motors and cleanup GPIO"""
        self.stop()
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    """Test the motors"""
    print("Testing Motor Controller...")
    print("Press Ctrl+C to exit\n")
    
    motors = MotorController()
    
    try:
        print("Moving forward...")
        motors.forward(speed=50)
        time.sleep(2)
        
        print("Stopping...")
        motors.stop()
        time.sleep(1)
        
        print("Moving backward...")
        motors.backward(speed=50)
        time.sleep(2)
        
        print("Stopping...")
        motors.stop()
        time.sleep(1)
        
        print("Turning left...")
        motors.turn_left()
        time.sleep(1)
        
        print("Stopping...")
        motors.stop()
        time.sleep(1)
        
        print("Turning right...")
        motors.turn_right()
        time.sleep(1)
        
        print("Stopping...")
        motors.stop()
        
    except KeyboardInterrupt:
        print("\n\nCleaning up...")
    finally:
        motors.cleanup()
        print("Done!")
