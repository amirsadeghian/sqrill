#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Configuration file for Autonomous RC Car
# GPIO Pin assignments and sensor/motor parameters

# Ultrasonic Sensor Pins (HC-SR04)
# Format: {'trigger': GPIO_PIN, 'echo': GPIO_PIN}
SENSOR_LEFT = {'trigger': 4, 'echo': 25}
SENSOR_CENTER = {'trigger': 23, 'echo': 24}
SENSOR_RIGHT = {'trigger': 17, 'echo': 16}

# Optional rear sensors (uncomment and set pins when added)
# SENSOR_BACK_LEFT = {'trigger': 5, 'echo': 6}
# SENSOR_BACK_RIGHT = {'trigger': 13, 'echo': 19}

# PIR Motion Detection Sensor (for squirrel detection)
PIR_SENSOR_PIN = 4  # GPIO pin for PIR sensor output
PIR_ENABLE = False  # Enable/disable PIR sensor (TEMPORARILY DISABLED)

# Motor Driver Pins - Using Single L298N H-Bridge for BOTH motors
# L298N Motor A → Drive Motor (forward/backward)
MOTOR_DRIVE_FORWARD = 26   # GPIO 26 → L298N IN1
MOTOR_DRIVE_BACKWARD = 22  # GPIO 22 → L298N IN2
MOTOR_DRIVE_PWM = 13       # GPIO 13 → L298N ENA

# L298N Motor B → Steering Motor (left/right)
MOTOR_STEER_LEFT = 6       # GPIO 6  → L298N IN3
MOTOR_STEER_RIGHT = 5      # GPIO 5  → L298N IN4
MOTOR_STEER_PWM = 12       # GPIO 12 → L298N ENB

MOTOR_DRIVER_TYPE = 'HBRIDGE'  # Using L298N H-bridge for both motors

# Camera Settings (Optimized for Raspberry Pi 2)
# For Pi 3/4: use (640, 480) @ 30fps
# For Pi 2/Zero: use (320, 240) @ 15fps for better performance
CAMERA_RESOLUTION = (320, 240)  # Lower resolution for older Pi
CAMERA_FRAMERATE = 15  # Reduced framerate for smoother processing

# Obstacle Detection Parameters
MIN_DISTANCE = 30  # Minimum distance in cm before taking action
SAFE_DISTANCE = 50  # Comfortable distance in cm
CRITICAL_DISTANCE = 15  # Emergency stop distance in cm

# Speed Settings (PWM duty cycle 0-100)
# Note: Increase if motor buzzes but doesn't move (needs more starting torque)
SPEED_NORMAL = 100  # Increased for better motor starting torque
SPEED_SLOW = 80    # Minimum speed that still moves the motor
SPEED_TURN = 85    # Speed during turns

# Sensor Reading
SENSOR_TIMEOUT = 0.5  # seconds
MAX_DISTANCE = 400  # Maximum reliable distance in cm

# Decision Making
SCAN_INTERVAL = 0.1  # Time between sensor readings in seconds
TURN_DURATION = 0.5  # Duration for turns in seconds

# Steering Motor Settings
STEERING_PULSE_DURATION = 0.3  # Time to apply steering (seconds) before motor stops
STEERING_HOLD_POSITION = True  # Keep direction pins set after pulse (holds position mechanically)

# Squirrel Detection Parameters
SQUIRREL_MODE = True  # Enable squirrel detection mode
MOTION_DETECTION_THRESHOLD = 25  # Minimum pixel change % to detect motion (camera)
MOTION_AREA_THRESHOLD = 500  # Minimum contour area for valid motion
CHASE_DURATION = 3.0  # How long to chase detected motion (seconds)
CHASE_SPEED = 80  # Speed when chasing squirrels
MOTION_COOLDOWN = 2.0  # Seconds to wait after chase before detecting again
PIR_DEBOUNCE_TIME = 0.5  # Seconds to debounce PIR sensor

# PlayStation Controller Settings
CONTROLLER_ENABLE = True  # Enable PS controller support
CONTROLLER_VENDOR_ID = "0810"  # USB Vendor ID for Dual PSX Adaptor
CONTROLLER_PRODUCT_ID = "0001"  # USB Product ID
CONTROLLER_SPEED = 70  # Speed when using controller (0-100)
CONTROLLER_TURN_SPEED = 60  # Turn speed when using controller
CONTROLLER_USE_ANALOG = True  # Use analog sticks (False = D-pad only)
CONTROLLER_DEADZONE = 20  # Analog stick deadzone (prevents drift)
