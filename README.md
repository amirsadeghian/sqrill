# 🚗 Autonomous RC Car

A Raspberry Pi-based autonomous RC car with ultrasonic sensors for obstacle avoidance, camera support, and **squirrel detection/chase mode**! 🐿️

## Features

- **Obstacle Avoidance**: 3 ultrasonic sensors (left, center, right) for 180° front coverage
- **🐿️ Squirrel Detection**: PIR motion sensor + camera-based motion detection
- **Chase Mode**: Automatically detects and chases squirrels or moving objects
- **Smart Navigation**: Analyzes sensor data to choose optimal path
- **Multiple Movement Modes**: 
  - Emergency stop
  - Reverse and turn
  - Sharp turns
  - Gentle corrections
  - Variable speed control
- **Camera Support**: Computer vision for motion detection and verification
- **Real-time Monitoring**: Live sensor readings and statistics

## Modes of Operation

### 1. � PlayStation Controller (NEW!)
Direct control with PS1/PS2 controller:
```bash
python controller_car.py
```

**Controls:**
- **D-Pad Up/Down**: Forward/Backward
- **D-Pad Left/Right**: Turn left/right
- **Left Analog Stick**: Smooth steering (if enabled)
- **X Button**: Emergency stop
- **START**: Exit

### 2. 🌐 Web Control Interface
Control your car from any device on WiFi:
```bash
python web_control.py
```
Then open in browser: `http://[raspberry-pi-ip]:5000`

**Features:**
- 📱 Works on phone, tablet, or computer
- 📷 Live camera feed
- 🎮 Touch/click controls or keyboard (WASD/Arrows)
- 🎮 **Auto-detects PS controller** (use alongside web interface!)
- 📊 Real-time sensor readings
- 🤖 Switch between Manual/Autonomous/Squirrel Chase modes
- 🔴 Emergency stop button

### 3. Autonomous Obstacle Avoidance
Basic autonomous driving with obstacle detection:
```bash
python autonomous_car.py
```

### 4. 🐿️ Squirrel Chaser Mode
Detects and chases squirrels using PIR sensor + camera:
```bash
python squirrel_chaser.py
```

Options:
- `--camera` or `-c`: Enable camera motion detection
- `--pir` or `-p`: Enable PIR sensor
- `--no-camera`: Disable camera (PIR only)
- `--no-pir`: Disable PIR (camera only)

## Hardware Requirements

### Raspberry Pi Compatibility
- **Raspberry Pi 2**: Works (optimized camera settings included)
- **Raspberry Pi 3/3B+**: Works great
- **Raspberry Pi 4**: Best performance
- **Raspberry Pi Zero 2W**: Works (limited processing power)
- **Python Version**: 3.8 or newer (3.9+ recommended)
- **OS**: Raspberry Pi OS Bullseye or newer

### Components
- Raspberry Pi (any model with GPIO)
- 3x HC-SR04 Ultrasonic Sensors
- **1x PIR Motion Sensor (HC-SR501 or similar)** 🆕
- L298N Motor Driver (or similar H-bridge)
- 2x DC Motors
- RC Car Chassis
- Power Supply (battery pack)
- Optional: Raspberry Pi Camera Module or USB Camera
- **Optional: PS1/PS2 Controller + USB Adapter (0810:0001)** 🎮

### Wiring

#### PIR Motion Sensor (HC-SR501) 🆕
- **VCC**: Connect to 5V
- **GND**: Connect to Ground
- **OUT**: Connect to GPIO4 (configurable in config.py)

#### Ultrasonic Sensors (HC-SR04)
Default GPIO pins (configurable in `config.py`):
- **Left Sensor**: Trigger=GPIO23, Echo=GPIO24
- **Center Sensor**: Trigger=GPIO17, Echo=GPIO27
- **Right Sensor**: Trigger=GPIO22, Echo=GPIO10

#### Motor Driver (L298N)
- **Drive Motor (Forward/Backward)**: PWM=GPIO13
- **Steering Motor**: Left=GPIO6, Right=GPIO5, PWM=GPIO12

**Note:** This setup is for RC cars with Ackermann steering (one drive motor + one steering motor), not tank-style differential drive.

### Circuit Diagram
```
Raspberry Pi                PIR Motion Sensor
    GPIO4  <──────────────── OUT
    5V ────────────────────> VCC
    GND ───────────────────> GND

Raspberry Pi                HC-SR04 Sensors
    GPIO23 ────────────────> Left Trigger
    GPIO24 <──────────────── Left Echo
    GPIO17 ────────────────> Center Trigger
    GPIO27 <──────────────── Center Echo
    GPIO22 ────────────────> Right Trigger
    GPIO10 <──────────────── Right Echo

Raspberry Pi                Motor Driver (L298N)
    GPIO13 ────────────────> Drive Motor PWM (speed control)
    GPIO6  ────────────────> Steering Left
    GPIO5  ────────────────> Steering Right  
    GPIO12 ────────────────> Steering Motor PWM
    
    5V ────────────────────> 5V (for logic)
    GND ───────────────────> GND
    
Note: Drive motor uses single PWM for speed. For bidirectional 
control (forward/backward), wire motor driver accordingly or
add direction pins in config.py if needed.
```

## Software Setup

### Requirements
- **Python 3.8 or newer** (Python 3.9+ recommended)
- Raspberry Pi OS (Bullseye or newer)

### 1. Install Raspberry Pi OS
```bash
# Update system
sudo apt update
sudo apt upgrade -y
```

### 2. Check Python Version
```bash
python3 --version
# Should show Python 3.8.0 or higher
```

### 3. Enable Camera (if using)
```bash
sudo raspi-config
# Navigate to: Interface Options > Camera > Enable
```

### 4. Install Python Dependencies

**Option A - Using pip (traditional):**
```bash
cd sqrill
pip install -r requirements.txt
```

**Option B - Using pyproject.toml (modern, Python 3.8+):**
```bash
cd sqrill
pip install -e .
```

**Option C - Install system packages first (recommended for Pi 2):**
```bash
# Install system dependencies
sudo apt-get install python3-opencv python3-numpy python3-flask

# Then install remaining Python packages
pip install -r requirements.txt
```

### 5. Configure GPIO Pins
Edit `config.py` to match your wiring:
```python
# Adjust these values based on your setup
SENSOR_LEFT = {'trigger': 23, 'echo': 24}
MOTOR_LEFT_FORWARD = 18
# etc...
```

### 6. Verify Installation ✨
Run the system check script to verify everything is working:
```bash
python3 check_system.py
```

This will check:
- ✅ Python version (>= 3.8)
- ✅ All dependencies installed
- ✅ GPIO permissions
- ✅ Camera availability (optional)
- ✅ Controller connection (optional)

## Usage

### Web Control (Recommended!)
```bash
# Start web server
python web_control.py

# Find your Raspberry Pi's IP address
hostname -I

# Access from any device:
# http://192.168.1.XXX:5000
```

**Keyboard Controls (in web browser):**
- `W` or `↑` - Forward
- `S` or `↓` - Backward  
- `A` or `←` - Left
- `D` or `→` - Right
- `Space` - Stop

### Basic Autonomous Mode (No Camera)
```bash
python autonomous_car.py
```

### With Camera Support
```bash
python autonomous_car.py --camera
```

### Test Individual Components

**Test PS Controller:** 🎮
```bash
python ps_controller.py
```

**Test PIR Motion Sensor:** 🆕
```bash
python motion_detection.py
```

**Test Sensors:**
```bash
python ultrasonic_sensors.py
```

**Test Motors:**
```bash
python motor_control.py
```

**Test Camera:**
```bash
python camera_module.py
```

## Configuration

All settings are in `config.py`:

### Distance Thresholds (in cm)
- `MIN_DISTANCE`: 30cm - triggers obstacle avoidance
- `SAFE_DISTANCE`: 50cm - comfortable operating distance
- `CRITICAL_DISTANCE`: 15cm - emergency stop

### Speed Settings (PWM duty cycle 0-100)
- `SPEED_NORMAL`: 70 - normal driving speed
- `SPEED_SLOW`: 40 - cautious speed
- `SPEED_TURN`: 60 - turning speed

### Timing
- `SCAN_INTERVAL`: 0.1 seconds between sensor readings
- `TURN_DURATION`: 0.5 seconds for sharp turns

### Squirrel Detection 🆕
- `SQUIRREL_MODE`: Enable/disable squirrel detection
- `MOTION_DETECTION_THRESHOLD`: Sensitivity for camera motion detection (%)
- `CHASE_DURATION`: How long to chase detected motion (3.0 seconds)
- `CHASE_SPEED`: Speed when chasing (80 PWM)
- `MOTION_COOLDOWN`: Wait time after chase (2.0 seconds)
- `PIR_DEBOUNCE_TIME`: Prevent false triggers (0.5 seconds)

### PS Controller 🎮
- `CONTROLLER_SPEED`: Speed when using controller (70 PWM)
- `CONTROLLER_TURN_SPEED`: Turn speed (60 PWM)
- `CONTROLLER_USE_ANALOG`: Use analog sticks vs D-pad only
- `CONTROLLER_DEADZONE`: Analog stick deadzone (prevents drift)

## How It Works

### Squirrel Detection Algorithm 🆕

**Detection Phase:**
1. **PIR Sensor**: Continuously monitors for infrared heat changes (animals)
2. **Camera Verification**: When PIR triggers, camera confirms motion with computer vision
3. **Direction Analysis**: Determines if squirrel is left, center, or right
4. **Chase Decision**: Evaluates if safe to chase (no obstacles)

**Chase Phase:**
1. **Orient**: Turn toward detected motion
2. **Pursue**: Drive forward at chase speed (80 PWM)
3. **Obstacle Check**: Continuously monitor ultrasonic sensors
4. **Duration**: Chase for 3 seconds or until obstacle
5. **Cooldown**: Wait 2 seconds before next detection

**Patrol Phase:**
- When no motion detected, slowly patrol area
- Use obstacle avoidance to navigate safely
- Continuously scan for new motion

### Obstacle Avoidance Algorithm

1. **Sensor Reading**: Continuously reads distances from all 3 sensors
2. **Situation Analysis**:
   - **Critical (< 15cm)**: Emergency stop + reverse
   - **Warning (< 30cm)**: Turn toward open space
   - **Caution (30-50cm)**: Gentle correction
   - **Safe (> 50cm)**: Full speed ahead

3. **Decision Making**:
   - Compares left vs right sensor readings
   - Chooses direction with most space
   - Adjusts speed based on proximity

4. **Execution**: Sends commands to motors

### Movement Modes

- **Forward**: Both motors forward
- **Backward**: Both motors reverse
- **Sharp Turn**: One motor forward, one reverse (pivot turn)
- **Gentle Turn**: Slow down one motor (gradual turn)
- **Stop**: Cut power to both motors

## Adding Rear Sensors

Uncomment and configure in `config.py`:
```python
SENSOR_BACK_LEFT = {'trigger': 5, 'echo': 6}
SENSOR_BACK_RIGHT = {'trigger': 13, 'echo': 19}
```

Update `ultrasonic_sensors.py` to add rear sensors to SensorArray.

## Future Enhancements

- [x] **Squirrel detection and chase mode** 🐿️
- [x] **PIR motion sensor integration**
- [x] **Camera-based motion detection**
- [x] **WiFi web control interface** 🌐
- [x] **Live camera streaming**
- [x] **Real-time sensor monitoring**
- [x] **PlayStation controller support** 🎮
- [x] **Python 3.8+ support with modern packaging** 🐍
- [x] **System check script for easy setup**
- [ ] Mobile app (iOS/Android)
- [ ] Voice control integration
- [ ] Xbox/other controller support
- [ ] Line following using camera
- [ ] Object recognition (stop signs, traffic lights)
- [ ] GPS waypoint navigation
- [ ] Remote control override via web interface
- [ ] Collision detection with IMU
- [ ] Path recording and replay
- [ ] Multiple animal detection (birds, cats, dogs)
- [ ] Sound detection (barking, chirping)

## Troubleshooting

### Sensors not working
- Check wiring connections
- Verify GPIO pin numbers in config.py
- Test with multimeter (VCC should be 5V)
- Run sensor test: `python ultrasonic_sensors.py`

### Motors not responding
- Check motor driver power supply (needs separate battery)
- Verify PWM pins are configured correctly
- Test motors directly with power supply
- Run motor test: `python motor_control.py`

### Car drives in circles
- Motors may have different speeds - adjust in code
- Check motor wiring (may be reversed)
- Calibrate PWM values in config.py

### Camera not found
- Enable camera: `sudo raspi-config`
- Check cable connection
- Test: `libcamera-hello` (for Pi Camera)
- Try USB camera mode in camera_module.py

### PIR sensor not triggering 🆕
- Check wiring (VCC to 5V, GND to GND, OUT to GPIO4)
- Wait 30 seconds after power-on for sensor to stabilize
- Adjust sensitivity potentiometer on PIR module
- Test with: `python motion_detection.py`
- Try adjusting `PIR_DEBOUNCE_TIME` in config.py

### False motion detections 🆕
- Increase `MOTION_DETECTION_THRESHOLD` in config.py
- Increase `MOTION_AREA_THRESHOLD` (ignores small movements)
- Position PIR sensor away from heat sources
- Ensure camera has stable mount (reduce vibration)
- Increase `MOTION_COOLDOWN` time

### Car chases everything, not just squirrels 🆕
- PIR sensors detect all warm-blooded animals
- Lower `MOTION_DETECTION_THRESHOLD` for more selective detection
- Add computer vision object detection (future enhancement)
- Position PIR sensor at squirrel height

### Can't access web interface 🆕
- Check Raspberry Pi is connected to WiFi
- Find IP with: `hostname -I`
- Make sure port 5000 is not blocked
- Try accessing from Pi first: `http://localhost:5000`
- Ensure Flask is installed: `pip install flask flask-socketio`

### Web interface is laggy 🆕
- Camera resolution already optimized for Pi 2
- Reduce to 160x120 in config.py for even faster streaming
- Reduce `CAMERA_FRAMERATE` to 10
- Use wired Ethernet instead of WiFi if possible
- Close other browser tabs

### PS Controller not detected 🎮
- Check adapter is plugged in: `lsusb | grep 0810`
- Verify controller is connected to adapter (LED should light up)
- Add user to input group: `sudo usermod -a -G input $USER`
- Then logout and login for changes to take effect
- Check permissions: `ls -l /dev/input/`
- Test controller: `python ps_controller.py`
- Try different USB port
- Some adapters need calibration - press SELECT+START

### Controller has drift/not responding 🎮
- Increase `CONTROLLER_DEADZONE` in config.py (try 30-40)
- Calibrate controller if adapter supports it
- Try D-pad only mode: set `CONTROLLER_USE_ANALOG = False`
- Check analog sticks are centered when idle
- Clean analog stick contacts (oxidation can cause issues)

### Python version too old 🐍
- Check version: `python3 --version`
- Need Python 3.8 or newer
- Update Python:
  ```bash
  sudo apt-get update
  sudo apt-get install python3.9 python3-pip
  python3.9 -m pip install -r requirements.txt
  ```
- Run with specific version: `python3.9 autonomous_car.py`
- Or update Raspberry Pi OS to newer version (Bullseye/Bookworm)

### Dependency installation fails 🐍
- Try system packages first (faster on Pi 2):
  ```bash
  sudo apt-get install python3-opencv python3-numpy python3-flask
  pip install -r requirements.txt
  ```
- If pip is old: `python3 -m pip install --upgrade pip`
- Check available space: `df -h`

## Safety Tips

⚠️ **Important Safety Information**

- Always test in a safe, enclosed area
- Have emergency stop ready (Ctrl+C or power switch)
- Start with slow speeds (30-40 PWM)
- Keep clear of obstacles during initial testing
- Disconnect motors during software testing
- Use appropriate power supply (don't overdraw from Pi's 5V)

## License

MIT License - feel free to modify and use for your projects!

## Contributing

Feel free to submit issues or pull requests for improvements!

## Author

Created for autonomous RC car enthusiasts and hobbyists.

---

Happy autonomous driving! 🚗💨
