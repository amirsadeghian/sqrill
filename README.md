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

### 1. 🌐 Web Control Interface (NEW!)
Control your car from any device on WiFi:
```bash
python web_control.py
```
Then open in browser: `http://[raspberry-pi-ip]:5000`

**Features:**
- 📱 Works on phone, tablet, or computer
- 📷 Live camera feed
- 🎮 Touch/click controls or keyboard (WASD/Arrows)
- 📊 Real-time sensor readings
- 🤖 Switch between Manual/Autonomous/Squirrel Chase modes
- 🔴 Emergency stop button

### 2. Autonomous Obstacle Avoidance
Basic autonomous driving with obstacle detection:
```bash
python autonomous_car.py
```

### 3. 🐿️ Squirrel Chaser Mode
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

### Components
- Raspberry Pi (any model with GPIO)
- 3x HC-SR04 Ultrasonic Sensors
- **1x PIR Motion Sensor (HC-SR501 or similar)** 🆕
- L298N Motor Driver (or similar H-bridge)
- 2x DC Motors
- RC Car Chassis
- Power Supply (battery pack)
- Optional: Raspberry Pi Camera Module or USB Camera

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
- **Left Motor**: Forward=GPIO18, Backward=GPIO15, Enable(PWM)=GPIO14
- **Right Motor**: Forward=GPIO7, Backward=GPIO8, Enable(PWM)=GPIO25

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

Raspberry Pi                L298N Motor Driver
    GPIO18 ────────────────> IN1 (Left Forward)
    GPIO15 ────────────────> IN2 (Left Backward)
    GPIO14 ────────────────> ENA (Left Enable/PWM)
    GPIO7  ────────────────> IN3 (Right Forward)
    GPIO8  ────────────────> IN4 (Right Backward)
    GPIO25 ────────────────> ENB (Right Enable/PWM)
    
    5V ────────────────────> 5V (for logic)
    GND ───────────────────> GND
```

## Software Setup

### 1. Install Raspberry Pi OS
```bash
# Update system
sudo apt update
sudo apt upgrade -y
```

### 2. Enable Camera (if using)
```bash
sudo raspi-config
# Navigate to: Interface Options > Camera > Enable
```

### 3. Install Python Dependencies
```bash
cd sqrill
pip install -r requirements.txt
```

### 4. Configure GPIO Pins
Edit `config.py` to match your wiring:
```python
# Adjust these values based on your setup
SENSOR_LEFT = {'trigger': 23, 'echo': 24}
MOTOR_LEFT_FORWARD = 18
# etc...
```

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
- [ ] Mobile app (iOS/Android)
- [ ] Voice control integration
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
