#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
    """Controls the motors for the RC car with steering motor"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup motor pins
        self.setup_pins()
        
        # Setup PWM for drive motor (forward/backward)
        self.pwm_drive = GPIO.PWM(config.MOTOR_DRIVE_PWM, 1000)  # 1kHz frequency
        self.pwm_drive.start(0)
        
        # Setup PWM for steering motor
        self.pwm_steer = GPIO.PWM(config.MOTOR_STEER_PWM, 1000)
        self.pwm_steer.start(0)
        
        self.current_speed = config.SPEED_NORMAL
        self.current_steering = 0  # -100 (full left) to +100 (full right), 0 = center
        self.is_moving = False
        
    def setup_pins(self):
        """Setup GPIO pins for motors"""
        pins = [
            config.MOTOR_DRIVE_PWM,
            config.MOTOR_STEER_LEFT,
            config.MOTOR_STEER_RIGHT,
            config.MOTOR_STEER_PWM
        ]
        
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)
    
    def set_speed(self, speed: int):
        """Set motor speed (0-100)"""
        self.current_speed = max(0, min(100, speed))
    
    def stop(self):
        """Stop drive motor and center steering"""
        self.pwm_drive.ChangeDutyCycle(0)
        self.center_steering()
        self.is_moving = False
    
    def forward(self, speed: int = None):
        """Move forward"""
        if speed is None:
            speed = self.current_speed
        
        # Drive motor forward (positive PWM)
        self.pwm_drive.ChangeDutyCycle(speed)
        self.is_moving = True
    
    def backward(self, speed: int = None):
        """Move backward"""
        if speed is None:
            speed = self.current_speed
        
        # Drive motor backward
        # NOTE: With only PWM pin, this needs proper H-bridge wiring
        # You may need to add MOTOR_DRIVE_FORWARD and MOTOR_DRIVE_BACKWARD pins
        # in config.py and control them here for direction control
        # For now, using PWM only (works if H-bridge is wired for bidirectional control)
        self.pwm_drive.ChangeDutyCycle(-speed if speed < 0 else speed)
        self.is_moving = True
    
    def steer_left(self, amount: int = 100):
        """
        Steer left
        amount: 0-100, where 100 is full left
        """
        amount = max(0, min(100, amount))
        self.current_steering = -amount
        
        GPIO.output(config.MOTOR_STEER_LEFT, True)
        GPIO.output(config.MOTOR_STEER_RIGHT, False)
        self.pwm_steer.ChangeDutyCycle(amount)
    
    def steer_right(self, amount: int = 100):
        """
        Steer right
        amount: 0-100, where 100 is full right
        """
        amount = max(0, min(100, amount))
        self.current_steering = amount
        
        GPIO.output(config.MOTOR_STEER_LEFT, False)
        GPIO.output(config.MOTOR_STEER_RIGHT, True)
        self.pwm_steer.ChangeDutyCycle(amount)
    
    def center_steering(self):
        """Center the steering (go straight)"""
        GPIO.output(config.MOTOR_STEER_LEFT, False)
        GPIO.output(config.MOTOR_STEER_RIGHT, False)
        self.pwm_steer.ChangeDutyCycle(0)
        self.current_steering = 0
    
    def turn_left(self, speed: int = None):
        """Turn left (steer left while moving forward)"""
        if speed is None:
            speed = config.SPEED_TURN
        
        self.steer_left(100)
        self.forward(speed)
    
    def turn_right(self, speed: int = None):
        """Turn right (steer right while moving forward)"""
        if speed is None:
            speed = config.SPEED_TURN
        
        self.steer_right(100)
        self.forward(speed)
    
    def gentle_left(self, speed: int = None):
        """Gentle left turn (partial steering)"""
        if speed is None:
            speed = self.current_speed
        
        self.steer_left(50)  # 50% steering
        self.forward(speed)
    
    def gentle_right(self, speed: int = None):
        """Gentle right turn (partial steering)"""
        if speed is None:
            speed = self.current_speed
        
        self.steer_right(50)  # 50% steering
        self.forward(speed)
    
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
        self.pwm_drive.stop()
        self.pwm_steer.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    """Test the motors"""
    print("Testing Motor Controller (Steering Setup)...")
    print("Press Ctrl+C to exit\n")
    
    motors = MotorController()
    
    try:
        print("Test 1: Moving forward...")
        motors.forward(speed=50)
        time.sleep(2)
        
        print("Stopping...")
        motors.stop()
        time.sleep(1)
        
        print("Test 2: Moving backward...")
        motors.backward(speed=50)
        time.sleep(2)
        
        print("Stopping...")
        motors.stop()
        time.sleep(1)
        
        print("Test 3: Steering left...")
        motors.steer_left(100)
        time.sleep(1)
        
        print("Centering steering...")
        motors.center_steering()
        time.sleep(1)
        
        print("Test 4: Steering right...")
        motors.steer_right(100)
        time.sleep(1)
        
        print("Centering steering...")
        motors.center_steering()
        time.sleep(1)
        
        print("Test 5: Turn left (forward + steer)...")
        motors.turn_left(speed=50)
        time.sleep(2)
        
        print("Stopping...")
        motors.stop()
        time.sleep(1)
        
        print("Test 6: Turn right (forward + steer)...")
        motors.turn_right(speed=50)
        time.sleep(2)
        
        print("Stopping...")
        motors.stop()
        
    except KeyboardInterrupt:
        print("\n\nCleaning up...")
    finally:
        motors.cleanup()
        print("Done!")
