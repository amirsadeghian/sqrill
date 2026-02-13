#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pin Testing Tool
Interactive tool to test and verify GPIO pin assignments
"""
import RPi.GPIO as GPIO
import time
import sys
import config


class PinTester:
    """Interactive pin testing tool"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.pwm_objects = {}
        
    def cleanup(self):
        """Cleanup GPIO"""
        for pwm in self.pwm_objects.values():
            pwm.stop()
        GPIO.cleanup()
    
    def show_pin_configuration(self):
        """Display all pin assignments from config"""
        print("\n" + "="*60)
        print("PIN CONFIGURATION FROM config.py")
        print("="*60)
        
        print("\n📡 ULTRASONIC SENSORS (HC-SR04):")
        print(f"  Left   - Trigger: GPIO {config.SENSOR_LEFT['trigger']:2d}  Echo: GPIO {config.SENSOR_LEFT['echo']:2d}")
        print(f"  Center - Trigger: GPIO {config.SENSOR_CENTER['trigger']:2d}  Echo: GPIO {config.SENSOR_CENTER['echo']:2d}")
        print(f"  Right  - Trigger: GPIO {config.SENSOR_RIGHT['trigger']:2d}  Echo: GPIO {config.SENSOR_RIGHT['echo']:2d}")
        
        print("\n🚗 DRIVE MOTOR:")
        driver_type = getattr(config, 'MOTOR_DRIVER_TYPE', 'HBRIDGE')
        print(f"  Driver Type:  {driver_type}")
        
        if driver_type == 'MOSFET':
            print(f"  PWM/Speed:    GPIO {config.MOTOR_DRIVE_PWM} (XY-MOS signal pin)")
            print(f"  Note: MOSFET mode only supports FORWARD direction")
        else:
            if hasattr(config, 'MOTOR_DRIVE_FORWARD'):
                print(f"  Forward Pin:  GPIO {config.MOTOR_DRIVE_FORWARD} (L298N IN1)")
            if hasattr(config, 'MOTOR_DRIVE_BACKWARD'):
                print(f"  Backward Pin: GPIO {config.MOTOR_DRIVE_BACKWARD} (L298N IN2)")
            print(f"  PWM/Speed:    GPIO {config.MOTOR_DRIVE_PWM} (L298N ENA)")
        
        print("\n🔄 STEERING MOTOR:")
        print(f"  Steer Left:   GPIO {config.MOTOR_STEER_LEFT}")
        print(f"  Steer Right:  GPIO {config.MOTOR_STEER_RIGHT}")
        print(f"  PWM/Speed:    GPIO {config.MOTOR_STEER_PWM}")
        
        print(f"\n📹 PIR SENSOR:")
        print(f"  PIR Pin:      GPIO {config.PIR_SENSOR_PIN} (Status: {'DISABLED' if not config.PIR_ENABLE else 'ENABLED'})")
        
        print("\n" + "="*60 + "\n")
    
    def test_single_pin(self, pin: int, pin_name: str, duration: float = 2.0):
        """Test a single digital pin"""
        print(f"\nTesting {pin_name} (GPIO {pin})...")
        try:
            GPIO.setup(pin, GPIO.OUT)
            print(f"  Setting HIGH for {duration}s...")
            GPIO.output(pin, True)
            time.sleep(duration)
            print(f"  Setting LOW...")
            GPIO.output(pin, False)
            print(f"  ✓ {pin_name} test complete")
            return True
        except Exception as e:
            print(f"  ✗ Error testing {pin_name}: {e}")
            return False
    
    def test_pwm_pin(self, pin: int, pin_name: str, duty_cycle: int = 50, duration: float = 2.0):
        """Test a PWM pin"""
        print(f"\nTesting {pin_name} (GPIO {pin}) with PWM at {duty_cycle}%...")
        try:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, 1000)  # 1kHz
            pwm.start(0)
            
            print(f"  Ramping up to {duty_cycle}%...")
            for dc in range(0, duty_cycle + 1, 5):
                pwm.ChangeDutyCycle(dc)
                time.sleep(0.05)
            
            print(f"  Holding at {duty_cycle}% for {duration}s...")
            time.sleep(duration)
            
            print(f"  Ramping down...")
            for dc in range(duty_cycle, -1, -5):
                pwm.ChangeDutyCycle(dc)
                time.sleep(0.05)
            
            pwm.stop()
            print(f"  ✓ {pin_name} PWM test complete")
            return True
        except Exception as e:
            print(f"  ✗ Error testing {pin_name}: {e}")
            return False
    
    def test_drive_motor_forward(self, speed: int = 70, duration: float = 2.0):
        """Test drive motor in forward direction"""
        print(f"\n🚗 Testing DRIVE MOTOR - FORWARD at {speed}% for {duration}s")
        print("   Motor should spin FORWARD")
        try:
            driver_type = getattr(config, 'MOTOR_DRIVER_TYPE', 'HBRIDGE')
            GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
            
            # Setup direction pins if H-bridge
            if driver_type == 'HBRIDGE' and hasattr(config, 'MOTOR_DRIVE_FORWARD'):
                GPIO.setup(config.MOTOR_DRIVE_FORWARD, GPIO.OUT)
                GPIO.setup(config.MOTOR_DRIVE_BACKWARD, GPIO.OUT)
                GPIO.output(config.MOTOR_DRIVE_FORWARD, True)
                GPIO.output(config.MOTOR_DRIVE_BACKWARD, False)
            
            # Apply PWM
            pwm = GPIO.PWM(config.MOTOR_DRIVE_PWM, 1000)
            pwm.start(0)
            pwm.ChangeDutyCycle(speed)
            
            time.sleep(duration)
            
            # Stop
            pwm.ChangeDutyCycle(0)
            if driver_type == 'HBRIDGE' and hasattr(config, 'MOTOR_DRIVE_FORWARD'):
                GPIO.output(config.MOTOR_DRIVE_FORWARD, False)
            pwm.stop()
            
            print("   ✓ Forward test complete")
            return True
        except Exception as e:
            print(f"   ✗ Error: {e}")
            return False
    
    def test_drive_motor_backward(self, speed: int = 70, duration: float = 2.0):
        """Test drive motor in backward direction"""
        driver_type = getattr(config, 'MOTOR_DRIVER_TYPE', 'HBRIDGE')
        
        if driver_type == 'MOSFET':
            print(f"\n⚠ BACKWARD not supported with MOSFET driver (XY-MOS)")
            print("   MOSFET can only control speed in one direction")
            print("   To add backward: Use H-bridge (L298N) or add 2nd MOSFET")
            return False
        
        print(f"\n🚗 Testing DRIVE MOTOR - BACKWARD at {speed}% for {duration}s")
        print("   Motor should spin BACKWARD")
        try:
            GPIO.setup(config.MOTOR_DRIVE_FORWARD, GPIO.OUT)
            GPIO.setup(config.MOTOR_DRIVE_BACKWARD, GPIO.OUT)
            GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
            
            # Set direction
            GPIO.output(config.MOTOR_DRIVE_FORWARD, False)
            GPIO.output(config.MOTOR_DRIVE_BACKWARD, True)
            
            # Apply PWM
            pwm = GPIO.PWM(config.MOTOR_DRIVE_PWM, 1000)
            pwm.start(0)
            pwm.ChangeDutyCycle(speed)
            
            time.sleep(duration)
            
            # Stop
            pwm.ChangeDutyCycle(0)
            GPIO.output(config.MOTOR_DRIVE_BACKWARD, False)
            pwm.stop()
            
            print("   ✓ Backward test complete")
            return True
        except Exception as e:
            print(f"   ✗ Error: {e}")
            return False
    
    def test_steering_motor(self, direction: str, speed: int = 70, duration: float = 2.0):
        """Test steering motor"""
        print(f"\n🔄 Testing STEERING MOTOR - {direction.upper()} at {speed}% for {duration}s")
        try:
            GPIO.setup(config.MOTOR_STEER_LEFT, GPIO.OUT)
            GPIO.setup(config.MOTOR_STEER_RIGHT, GPIO.OUT)
            GPIO.setup(config.MOTOR_STEER_PWM, GPIO.OUT)
            
            # Set direction
            if direction.lower() == "left":
                GPIO.output(config.MOTOR_STEER_LEFT, True)
                GPIO.output(config.MOTOR_STEER_RIGHT, False)
            else:
                GPIO.output(config.MOTOR_STEER_LEFT, False)
                GPIO.output(config.MOTOR_STEER_RIGHT, True)
            
            # Apply PWM
            pwm = GPIO.PWM(config.MOTOR_STEER_PWM, 1000)
            pwm.start(0)
            pwm.ChangeDutyCycle(speed)
            
            time.sleep(duration)
            
            # Stop
            pwm.ChangeDutyCycle(0)
            GPIO.output(config.MOTOR_STEER_LEFT, False)
            GPIO.output(config.MOTOR_STEER_RIGHT, False)
            pwm.stop()
            
            print(f"   ✓ {direction} test complete")
            return True
        except Exception as e:
            print(f"   ✗ Error: {e}")
            return False
    
    def test_ultrasonic_sensor(self, sensor_name: str, sensor_config: dict):
        """Test an ultrasonic sensor"""
        print(f"\n📡 Testing {sensor_name} ULTRASONIC SENSOR")
        print(f"   Trigger: GPIO {sensor_config['trigger']}, Echo: GPIO {sensor_config['echo']}")
        
        try:
            GPIO.setup(sensor_config['trigger'], GPIO.OUT)
            GPIO.setup(sensor_config['echo'], GPIO.IN)
            
            # Send trigger pulse
            GPIO.output(sensor_config['trigger'], False)
            time.sleep(0.1)
            GPIO.output(sensor_config['trigger'], True)
            time.sleep(0.00001)
            GPIO.output(sensor_config['trigger'], False)
            
            # Wait for echo
            timeout = time.time() + 0.5
            pulse_start = time.time()
            while GPIO.input(sensor_config['echo']) == 0:
                pulse_start = time.time()
                if time.time() > timeout:
                    print("   ⚠ No echo received (timeout)")
                    return False
            
            pulse_end = time.time()
            while GPIO.input(sensor_config['echo']) == 1:
                pulse_end = time.time()
                if time.time() > timeout:
                    print("   ⚠ Echo never ended (timeout)")
                    return False
            
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound
            distance = round(distance, 2)
            
            print(f"   ✓ Distance measured: {distance} cm")
            return True
            
        except Exception as e:
            print(f"   ✗ Error: {e}")
            return False
    
    def interactive_menu(self):
        """Main interactive menu"""
        while True:
            print("\n" + "="*60)
            print("PIN TESTING MENU")
            print("="*60)
            print("1. Show pin configuration")
            print("2. Test drive motor - FORWARD")
            print("3. Test drive motor - BACKWARD")
            print("4. Test steering motor - LEFT")
            print("5. Test steering motor - RIGHT")
            print("6. Test all ultrasonic sensors")
            print("7. Test individual pins (drive motor)")
            print("8. Test individual pins (steering motor)")
            print("9. Custom speed test")
            print("0. Exit")
            print("="*60)
            
            choice = input("\nEnter choice: ").strip()
            
            if choice == "0":
                break
            elif choice == "1":
                self.show_pin_configuration()
            elif choice == "2":
                self.test_drive_motor_forward()
            elif choice == "3":
                self.test_drive_motor_backward()
            elif choice == "4":
                self.test_steering_motor("left")
            elif choice == "5":
                self.test_steering_motor("right")
            elif choice == "6":
                self.test_ultrasonic_sensor("LEFT", config.SENSOR_LEFT)
                time.sleep(0.5)
                self.test_ultrasonic_sensor("CENTER", config.SENSOR_CENTER)
                time.sleep(0.5)
                self.test_ultrasonic_sensor("RIGHT", config.SENSOR_RIGHT)
            elif choice == "7":
                driver_type = getattr(config, 'MOTOR_DRIVER_TYPE', 'HBRIDGE')
                print(f"\nTesting drive motor pins individually ({driver_type} mode):")
                
                if driver_type == 'HBRIDGE' and hasattr(config, 'MOTOR_DRIVE_FORWARD'):
                    self.test_single_pin(config.MOTOR_DRIVE_FORWARD, "FORWARD pin (IN1)", 2.0)
                    time.sleep(1)
                    self.test_single_pin(config.MOTOR_DRIVE_BACKWARD, "BACKWARD pin (IN2)", 2.0)
                    time.sleep(1)
                
                self.test_pwm_pin(config.MOTOR_DRIVE_PWM, "PWM pin (XY-MOS signal)" if driver_type == 'MOSFET' else "PWM pin (ENA)", 70, 2.0)
            elif choice == "8":
                print("\nTesting steering motor pins individually:")
                self.test_single_pin(config.MOTOR_STEER_LEFT, "STEER LEFT pin", 2.0)
                time.sleep(1)
                self.test_single_pin(config.MOTOR_STEER_RIGHT, "STEER RIGHT pin", 2.0)
                time.sleep(1)
                self.test_pwm_pin(config.MOTOR_STEER_PWM, "STEER PWM pin", 70, 2.0)
            elif choice == "9":
                try:
                    speed = int(input("Enter speed (0-100): "))
                    duration = float(input("Enter duration (seconds): "))
                    direction = input("Direction (forward/backward): ").lower()
                    
                    if direction == "forward":
                        self.test_drive_motor_forward(speed, duration)
                    else:
                        self.test_drive_motor_backward(speed, duration)
                except ValueError:
                    print("Invalid input")
            else:
                print("Invalid choice")
            
            input("\nPress Enter to continue...")


def main():
    """Main function"""
    print("\n" + "="*60)
    print("RC CAR PIN TESTING TOOL")
    print("="*60)
    print("\nThis tool helps verify GPIO pin assignments and test motors")
    print("WARNING: Ensure car is on blocks or wheels can spin freely!")
    print()
    
    ready = input("Is the car ready for testing? (y/n): ").strip().lower()
    if ready != 'y':
        print("Exiting...")
        return
    
    tester = PinTester()
    
    try:
        tester.show_pin_configuration()
        tester.interactive_menu()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        print("\nCleaning up GPIO...")
        tester.cleanup()
        print("Done!")


if __name__ == "__main__":
    main()
