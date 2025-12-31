"""
Web Control Interface for RC Car
Provides web-based control, live camera feed, and sensor monitoring
"""
from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO, emit
import cv2
import time
import threading
import json
from typing import Optional
import config
from ultrasonic_sensors import SensorArray
from motor_control import MotorController, Direction
from camera_module import CameraModule
from motion_detection import MotionDetectionManager


app = Flask(__name__)
app.config['SECRET_KEY'] = 'sqrill_secret_key_2025'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global car control instance
car_controller = None


class WebControlledCar:
    """Car controller with web interface support"""
    
    def __init__(self):
        print("Initializing Web-Controlled RC Car...")
        
        # Initialize components
        self.sensors = SensorArray()
        self.motors = MotorController()
        self.camera = None
        self.motion_detector = None
        
        # Control state
        self.mode = 'manual'  # manual, autonomous, squirrel_chase
        self.running = False
        self.autonomous_thread = None
        
        # Sensor data
        self.latest_distances = {'left': 0, 'center': 0, 'right': 0}
        self.motion_detected = False
        
        print("Web-Controlled RC Car ready!")
    
    def initialize_camera(self):
        """Initialize camera (lazy loading)"""
        if self.camera is None:
            try:
                self.camera = CameraModule(use_picamera=True)
                print("Camera initialized")
            except Exception as e:
                print(f"Camera initialization failed: {e}")
    
    def initialize_motion_detector(self):
        """Initialize motion detector (lazy loading)"""
        if self.motion_detector is None:
            try:
                self.motion_detector = MotionDetectionManager(enable_pir=True, enable_camera=True)
                print("Motion detector initialized")
            except Exception as e:
                print(f"Motion detector initialization failed: {e}")
    
    def get_camera_frame(self):
        """Get current camera frame"""
        if self.camera is None:
            self.initialize_camera()
        
        if self.camera:
            frame = self.camera.capture_frame()
            if frame is not None:
                # Add sensor overlay
                frame = self.camera.get_frame_with_overlay(frame, self.latest_distances)
                return frame
        return None
    
    def update_sensor_data(self):
        """Read and update sensor data"""
        self.latest_distances = self.sensors.get_safe_distances()
    
    def move(self, direction: str, duration: float = 0):
        """Move car in specified direction"""
        if direction == 'forward':
            self.motors.forward(speed=config.SPEED_NORMAL)
        elif direction == 'backward':
            self.motors.backward(speed=config.SPEED_NORMAL)
        elif direction == 'left':
            self.motors.turn_left(speed=config.SPEED_TURN)
        elif direction == 'right':
            self.motors.turn_right(speed=config.SPEED_TURN)
        elif direction == 'stop':
            self.motors.stop()
        
        if duration > 0:
            time.sleep(duration)
            self.motors.stop()
    
    def set_speed(self, speed: int):
        """Set motor speed (0-100)"""
        self.motors.set_speed(speed)
    
    def start_autonomous_mode(self):
        """Start autonomous obstacle avoidance"""
        self.mode = 'autonomous'
        self.running = True
        
        def autonomous_loop():
            from autonomous_car import AutonomousCar
            autonomous = AutonomousCar(enable_camera=False)
            
            while self.running and self.mode == 'autonomous':
                analysis = autonomous.analyze_sensors()
                autonomous.execute_action(analysis)
                self.latest_distances = analysis['distances']
                time.sleep(config.SCAN_INTERVAL)
            
            autonomous.motors.stop()
        
        self.autonomous_thread = threading.Thread(target=autonomous_loop, daemon=True)
        self.autonomous_thread.start()
    
    def start_squirrel_chase_mode(self):
        """Start squirrel detection and chase mode"""
        self.mode = 'squirrel_chase'
        self.running = True
        self.initialize_camera()
        self.initialize_motion_detector()
        
        def squirrel_loop():
            from squirrel_chaser import SquirrelChaserCar
            chaser = SquirrelChaserCar(enable_camera=True, enable_pir=True)
            
            while self.running and self.mode == 'squirrel_chase':
                motion_event = chaser.detect_squirrel()
                
                if motion_event and not chaser.in_chase:
                    chaser.chase_squirrel(motion_event)
                    self.motion_detected = True
                else:
                    chaser.patrol_mode()
                    self.motion_detected = False
                
                self.latest_distances = chaser.sensors.get_safe_distances()
                time.sleep(0.1)
            
            chaser.motors.stop()
        
        self.autonomous_thread = threading.Thread(target=squirrel_loop, daemon=True)
        self.autonomous_thread.start()
    
    def stop_autonomous(self):
        """Stop autonomous mode"""
        self.running = False
        self.mode = 'manual'
        self.motors.stop()
        time.sleep(0.2)
    
    def cleanup(self):
        """Clean up all resources"""
        self.running = False
        self.motors.cleanup()
        self.sensors.cleanup()
        if self.camera:
            self.camera.cleanup()
        if self.motion_detector:
            self.motion_detector.cleanup()


def generate_frames():
    """Generate video frames for streaming"""
    global car_controller
    
    while True:
        try:
            frame = car_controller.get_camera_frame()
            
            if frame is not None:
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            else:
                time.sleep(0.1)
        except Exception as e:
            print(f"Frame generation error: {e}")
            time.sleep(0.5)


# Flask Routes
@app.route('/')
def index():
    """Main control page"""
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/sensors')
def get_sensors():
    """Get current sensor readings"""
    global car_controller
    car_controller.update_sensor_data()
    return jsonify({
        'distances': car_controller.latest_distances,
        'motion_detected': car_controller.motion_detected,
        'mode': car_controller.mode
    })


@app.route('/api/control', methods=['POST'])
def control():
    """Handle control commands"""
    global car_controller
    
    data = request.json
    command = data.get('command')
    
    if command == 'move':
        direction = data.get('direction')
        duration = data.get('duration', 0)
        car_controller.move(direction, duration)
        return jsonify({'status': 'ok', 'command': command})
    
    elif command == 'speed':
        speed = data.get('speed', 50)
        car_controller.set_speed(speed)
        return jsonify({'status': 'ok', 'speed': speed})
    
    elif command == 'mode':
        mode = data.get('mode')
        
        if mode == 'manual':
            car_controller.stop_autonomous()
        elif mode == 'autonomous':
            car_controller.start_autonomous_mode()
        elif mode == 'squirrel_chase':
            car_controller.start_squirrel_chase_mode()
        
        return jsonify({'status': 'ok', 'mode': mode})
    
    return jsonify({'status': 'error', 'message': 'Unknown command'})


# SocketIO Events (for real-time updates)
@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print('Client connected')
    emit('status', {'message': 'Connected to RC Car'})


@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print('Client disconnected')


@socketio.on('control_command')
def handle_control(data):
    """Handle real-time control commands"""
    global car_controller
    
    command = data.get('command')
    direction = data.get('direction')
    
    if command == 'move':
        car_controller.move(direction)
        emit('status', {'command': command, 'direction': direction})
    elif command == 'stop':
        car_controller.move('stop')
        emit('status', {'command': 'stop'})


def background_sensor_updates():
    """Send sensor updates to all connected clients"""
    global car_controller
    
    while True:
        try:
            car_controller.update_sensor_data()
            socketio.emit('sensor_update', {
                'distances': car_controller.latest_distances,
                'mode': car_controller.mode,
                'motion': car_controller.motion_detected
            })
            time.sleep(0.2)
        except Exception as e:
            print(f"Sensor update error: {e}")
            time.sleep(1)


if __name__ == '__main__':
    print("=" * 60)
    print("🌐 RC CAR WEB CONTROL INTERFACE")
    print("=" * 60)
    
    # Initialize car controller
    car_controller = WebControlledCar()
    
    # Start background sensor updates
    sensor_thread = threading.Thread(target=background_sensor_updates, daemon=True)
    sensor_thread.start()
    
    # Get local IP address
    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    
    print(f"\n🚀 Server starting...")
    print(f"📱 Access from any device on your network:")
    print(f"   http://{local_ip}:5000")
    print(f"   http://localhost:5000 (from Pi)")
    print("\nPress Ctrl+C to stop\n")
    print("=" * 60)
    
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        car_controller.cleanup()
        print("Done!")
