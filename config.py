# Configuration file for Autonomous RC Car
# GPIO Pin assignments and sensor/motor parameters

# Ultrasonic Sensor Pins (HC-SR04)
# Format: {'trigger': GPIO_PIN, 'echo': GPIO_PIN}
SENSOR_LEFT = {'trigger': 23, 'echo': 24}
SENSOR_CENTER = {'trigger': 17, 'echo': 27}
SENSOR_RIGHT = {'trigger': 22, 'echo': 10}

# Optional rear sensors (uncomment and set pins when added)
# SENSOR_BACK_LEFT = {'trigger': 5, 'echo': 6}
# SENSOR_BACK_RIGHT = {'trigger': 13, 'echo': 19}

# PIR Motion Detection Sensor (for squirrel detection)
PIR_SENSOR_PIN = 4  # GPIO pin for PIR sensor output
PIR_ENABLE = True  # Enable/disable PIR sensor

# Motor Driver Pins (L298N or similar)
# Left Motor
MOTOR_LEFT_FORWARD = 18
MOTOR_LEFT_BACKWARD = 15
MOTOR_LEFT_ENABLE = 14  # PWM pin for speed control

# Right Motor
MOTOR_RIGHT_FORWARD = 7
MOTOR_RIGHT_BACKWARD = 8
MOTOR_RIGHT_ENABLE = 25  # PWM pin for speed control

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
SPEED_NORMAL = 70
SPEED_SLOW = 40
SPEED_TURN = 60

# Sensor Reading
SENSOR_TIMEOUT = 0.5  # seconds
MAX_DISTANCE = 400  # Maximum reliable distance in cm

# Decision Making
SCAN_INTERVAL = 0.1  # Time between sensor readings in seconds
TURN_DURATION = 0.5  # Duration for turns in seconds

# Squirrel Detection Parameters
SQUIRREL_MODE = True  # Enable squirrel detection mode
MOTION_DETECTION_THRESHOLD = 25  # Minimum pixel change % to detect motion (camera)
MOTION_AREA_THRESHOLD = 500  # Minimum contour area for valid motion
CHASE_DURATION = 3.0  # How long to chase detected motion (seconds)
CHASE_SPEED = 80  # Speed when chasing squirrels
MOTION_COOLDOWN = 2.0  # Seconds to wait after chase before detecting again
PIR_DEBOUNCE_TIME = 0.5  # Seconds to debounce PIR sensor
