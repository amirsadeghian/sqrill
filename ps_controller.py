"""
PlayStation Controller Module
Handles input from PS1/PS2 controllers via USB adapter
Supports Dual PSX Adaptor (0810:0001)
"""
import time
import threading
from typing import Optional, Callable, Dict
try:
    from evdev import InputDevice, categorize, ecodes, list_devices
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False
    print("Warning: evdev not available. Install with: pip install evdev")

import config


class PSController:
    """PlayStation Controller Handler"""
    
    # Button mappings for Dual PSX Adaptor
    BUTTON_MAP = {
        # D-Pad
        'DPAD_UP': 'BTN_TRIGGER',
        'DPAD_DOWN': 'BTN_THUMB',
        'DPAD_LEFT': 'BTN_THUMB2',
        'DPAD_RIGHT': 'BTN_TOP',
        
        # Face buttons (X, O, Square, Triangle)
        'X': 'BTN_TOP2',
        'CIRCLE': 'BTN_PINKIE',
        'SQUARE': 'BTN_BASE',
        'TRIANGLE': 'BTN_BASE2',
        
        # Shoulder buttons
        'L1': 'BTN_BASE3',
        'R1': 'BTN_BASE4',
        'L2': 'BTN_BASE5',
        'R2': 'BTN_BASE6',
        
        # Select/Start
        'SELECT': 'BTN_DEAD',
        'START': 'BTN_THUMBL'
    }
    
    # Alternative: Axis-based D-Pad (some adapters use this)
    AXIS_THRESHOLD = 127
    
    def __init__(self, vendor_id: str = "0810", product_id: str = "0001"):
        """
        Initialize PS controller
        Args:
            vendor_id: USB vendor ID (default: 0810 for PCS Inc)
            product_id: USB product ID (default: 0001 for Dual PSX Adaptor)
        """
        self.device = None
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.running = False
        
        # Controller state
        self.buttons = {}
        self.axes = {'x': 0, 'y': 0, 'rx': 0, 'ry': 0}  # Left stick (x,y), Right stick (rx,ry)
        
        # Callbacks
        self.on_button_press = None
        self.on_axis_move = None
        
        if not EVDEV_AVAILABLE:
            print("Cannot initialize controller - evdev library not installed")
            return
        
        # Find and connect to controller
        self.connect()
    
    def connect(self) -> bool:
        """Find and connect to the PS controller"""
        if not EVDEV_AVAILABLE:
            return False
        
        devices = [InputDevice(path) for path in list_devices()]
        
        for device in devices:
            # Check by vendor/product ID
            if (hex(device.info.vendor)[2:].zfill(4) == self.vendor_id and 
                hex(device.info.product)[2:].zfill(4) == self.product_id):
                self.device = device
                print(f"✅ PS Controller connected: {device.name}")
                print(f"   Device path: {device.path}")
                return True
            
            # Also check by name (fallback)
            if "dual" in device.name.lower() and "psx" in device.name.lower():
                self.device = device
                print(f"✅ PS Controller connected: {device.name}")
                print(f"   Device path: {device.path}")
                return True
        
        print("❌ PS Controller not found!")
        print(f"   Looking for: {self.vendor_id}:{self.product_id}")
        print("\nAvailable devices:")
        for device in devices:
            vendor = hex(device.info.vendor)[2:].zfill(4)
            product = hex(device.info.product)[2:].zfill(4)
            print(f"   {vendor}:{product} - {device.name}")
        
        return False
    
    def read_events(self):
        """Read controller events (blocking)"""
        if not self.device:
            return
        
        self.running = True
        
        try:
            for event in self.device.read_loop():
                if not self.running:
                    break
                
                # Button events
                if event.type == ecodes.EV_KEY:
                    button_name = self.get_button_name(event.code)
                    pressed = event.value == 1
                    
                    self.buttons[button_name] = pressed
                    
                    if self.on_button_press and pressed:
                        self.on_button_press(button_name)
                
                # Axis events (analog sticks, D-pad on some adapters)
                elif event.type == ecodes.EV_ABS:
                    axis_name = ecodes.ABS[event.code]
                    value = event.value
                    
                    # Map axis codes to stick names
                    if axis_name == 'ABS_X':
                        self.axes['x'] = value
                    elif axis_name == 'ABS_Y':
                        self.axes['y'] = value
                    elif axis_name == 'ABS_RX' or axis_name == 'ABS_Z':
                        self.axes['rx'] = value
                    elif axis_name == 'ABS_RY' or axis_name == 'ABS_RZ':
                        self.axes['ry'] = value
                    elif axis_name == 'ABS_HAT0X':  # D-pad X
                        if value < 0:
                            self.buttons['DPAD_LEFT'] = True
                        elif value > 0:
                            self.buttons['DPAD_RIGHT'] = True
                    elif axis_name == 'ABS_HAT0Y':  # D-pad Y
                        if value < 0:
                            self.buttons['DPAD_UP'] = True
                        elif value > 0:
                            self.buttons['DPAD_DOWN'] = True
                    
                    if self.on_axis_move:
                        self.on_axis_move(axis_name, value)
        
        except OSError as e:
            print(f"Controller disconnected: {e}")
            self.device = None
    
    def get_button_name(self, code: int) -> str:
        """Convert button code to readable name"""
        try:
            return ecodes.BTN[code]
        except KeyError:
            return f"BUTTON_{code}"
    
    def is_button_pressed(self, button: str) -> bool:
        """Check if a specific button is currently pressed"""
        return self.buttons.get(button, False)
    
    def get_axis_value(self, axis: str) -> int:
        """Get current value of analog stick axis"""
        return self.axes.get(axis, 128)
    
    def start_reading(self):
        """Start reading controller events in background thread"""
        if not self.device:
            return False
        
        thread = threading.Thread(target=self.read_events, daemon=True)
        thread.start()
        return True
    
    def stop(self):
        """Stop reading controller events"""
        self.running = False
    
    def cleanup(self):
        """Clean up controller"""
        self.stop()
        if self.device:
            self.device.close()


class PSControllerCarInterface:
    """Interface between PS controller and RC car"""
    
    def __init__(self, motor_controller):
        """
        Initialize controller interface
        Args:
            motor_controller: MotorController instance
        """
        self.controller = PSController()
        self.motors = motor_controller
        self.running = False
        
        # Control settings
        self.speed = config.CONTROLLER_SPEED
        self.turn_speed = config.CONTROLLER_TURN_SPEED
        self.analog_deadzone = config.CONTROLLER_DEADZONE
        
        # Control mode
        self.use_analog_sticks = config.CONTROLLER_USE_ANALOG
    
    def process_dpad_input(self):
        """Process D-pad button input"""
        # Priority: forward/backward over turning
        if self.controller.is_button_pressed('DPAD_UP') or \
           self.controller.is_button_pressed('BTN_TRIGGER'):
            self.motors.forward(speed=self.speed)
        elif self.controller.is_button_pressed('DPAD_DOWN') or \
             self.controller.is_button_pressed('BTN_THUMB'):
            self.motors.backward(speed=self.speed)
        elif self.controller.is_button_pressed('DPAD_LEFT') or \
             self.controller.is_button_pressed('BTN_THUMB2'):
            self.motors.turn_left(speed=self.turn_speed)
        elif self.controller.is_button_pressed('DPAD_RIGHT') or \
             self.controller.is_button_pressed('BTN_TOP'):
            self.motors.turn_right(speed=self.turn_speed)
        else:
            self.motors.stop()
    
    def process_analog_input(self):
        """Process analog stick input"""
        # Get left stick values (0-255, center at 128)
        x_axis = self.controller.get_axis_value('x')
        y_axis = self.controller.get_axis_value('y')
        
        # Calculate from center (128)
        x_delta = x_axis - 128
        y_delta = y_axis - 128
        
        # Apply deadzone
        if abs(x_delta) < self.analog_deadzone:
            x_delta = 0
        if abs(y_delta) < self.analog_deadzone:
            y_delta = 0
        
        # No input - stop
        if x_delta == 0 and y_delta == 0:
            self.motors.stop()
            return
        
        # Calculate motor speeds based on stick position
        # Forward/backward is primary, left/right adjusts
        forward_speed = -y_delta / 128.0  # Negative because up is 0
        turn_amount = x_delta / 128.0
        
        # Calculate left and right motor speeds
        left_speed = forward_speed + turn_amount
        right_speed = forward_speed - turn_amount
        
        # Normalize to -1.0 to 1.0
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1.0:
            left_speed /= max_speed
            right_speed /= max_speed
        
        # Convert to PWM values
        left_pwm = int(abs(left_speed) * self.speed)
        right_pwm = int(abs(right_speed) * self.speed)
        
        # Apply motor commands
        if left_speed > 0.1:
            self.motors.pwm_left.ChangeDutyCycle(left_pwm)
        elif left_speed < -0.1:
            self.motors.pwm_left.ChangeDutyCycle(left_pwm)
        
        if right_speed > 0.1:
            self.motors.pwm_right.ChangeDutyCycle(right_pwm)
        elif right_speed < -0.1:
            self.motors.pwm_right.ChangeDutyCycle(right_pwm)
    
    def run(self):
        """Main control loop"""
        if not self.controller.device:
            print("❌ Controller not connected!")
            return
        
        print("\n" + "=" * 60)
        print("🎮 PS CONTROLLER MODE ACTIVE")
        print("=" * 60)
        print("Controls:")
        print("  D-Pad / Left Stick: Move car")
        print("  X Button: Stop")
        print("  START: Exit")
        print("=" * 60)
        print("\nReady! Use controller to drive.\n")
        
        self.running = True
        self.controller.start_reading()
        
        try:
            while self.running:
                # Check for exit (START button)
                if self.controller.is_button_pressed('BTN_THUMBL') or \
                   self.controller.is_button_pressed('START'):
                    print("\nSTART button pressed - exiting...")
                    break
                
                # Emergency stop (X button)
                if self.controller.is_button_pressed('BTN_TOP2') or \
                   self.controller.is_button_pressed('X'):
                    self.motors.stop()
                    time.sleep(0.1)
                    continue
                
                # Process input based on mode
                if self.use_analog_sticks:
                    self.process_analog_input()
                else:
                    self.process_dpad_input()
                
                time.sleep(0.05)  # 20Hz update rate
        
        except KeyboardInterrupt:
            print("\n\nInterrupted!")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\n🧹 Cleaning up...")
        self.running = False
        self.motors.stop()
        self.controller.cleanup()
        print("✅ Done!")


if __name__ == "__main__":
    """Test PS controller"""
    print("Testing PS Controller...")
    print("Press Ctrl+C to exit\n")
    
    controller = PSController()
    
    if not controller.device:
        print("\n❌ Could not connect to controller!")
        print("Make sure:")
        print("1. Controller adapter is plugged in")
        print("2. Controller is connected to adapter")
        print("3. You have permissions to access /dev/input devices")
        print("\nTry: sudo usermod -a -G input $USER")
        print("Then log out and back in")
        exit(1)
    
    def on_button(button_name):
        print(f"Button pressed: {button_name}")
    
    controller.on_button_press = on_button
    
    print("Press buttons on controller...")
    print("Press START to exit\n")
    
    try:
        controller.start_reading()
        
        while controller.running:
            # Display current state
            pressed = [btn for btn, state in controller.buttons.items() if state]
            if pressed:
                print(f"\rPressed: {', '.join(pressed)}", end='', flush=True)
            
            # Check for START to exit
            if controller.is_button_pressed('BTN_THUMBL'):
                print("\n\nSTART pressed - exiting...")
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nExiting...")
    finally:
        controller.cleanup()
