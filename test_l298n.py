#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L298N H-Bridge Diagnostic Tool
Helps diagnose why L298N is outputting only 0.25V
"""
import RPi.GPIO as GPIO
import time
import config


def show_checklist():
    """Show pre-test checklist"""
    print("\n" + "="*60)
    print("L298N WIRING CHECKLIST")
    print("="*60)
    
    print("\n⚡ POWER CONNECTIONS:")
    print("  ☐ Battery (+) 6-12V → L298N +12V pin")
    print("  ☐ Battery (-) → L298N GND")
    print("  ☐ Pi GND → L298N GND (CRITICAL - common ground!)")
    print("  ☐ Battery voltage measured: ___V (should be 6-12V)")
    
    print("\n🔌 SIGNAL CONNECTIONS:")
    print("  ☐ Pi GPIO 26 → L298N IN1")
    print("  ☐ Pi GPIO 22 → L298N IN2")
    print("  ☐ Pi GPIO 13 → L298N ENA")
    
    print("\n⚙️  JUMPER SETTINGS:")
    print("  ☐ ENA jumper REMOVED (critical for PWM control)")
    print("  ☐ ENB jumper REMOVED")
    print("  ☐ +5V jumper status: check if L298N has one")
    
    print("\n🔋 MOTOR CONNECTIONS:")
    print("  ☐ Motor wire 1 → L298N OUT1")
    print("  ☐ Motor wire 2 → L298N OUT2")
    
    print("\n" + "="*60)


def test_gpio_outputs():
    """Test that GPIO pins can output correctly"""
    print("\n" + "="*60)
    print("GPIO OUTPUT TEST")
    print("="*60)
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    pins = {
        'IN1 (Forward)': config.MOTOR_DRIVE_FORWARD,
        'IN2 (Backward)': config.MOTOR_DRIVE_BACKWARD,
        'ENA (PWM)': config.MOTOR_DRIVE_PWM
    }
    
    print("\nMeasure voltage at each GPIO pin (relative to GND):")
    print("Expected: 3.3V when HIGH, 0V when LOW\n")
    
    for name, pin in pins.items():
        GPIO.setup(pin, GPIO.OUT)
        
        print(f"\n{name} (GPIO {pin}):")
        print("  Setting LOW...")
        GPIO.output(pin, False)
        input(f"  Measure voltage: ___V (should be ~0V). Press Enter...")
        
        print("  Setting HIGH...")
        GPIO.output(pin, True)
        input(f"  Measure voltage: ___V (should be ~3.3V). Press Enter...")
        
        GPIO.output(pin, False)
    
    GPIO.cleanup()


def test_forward_steady():
    """Test forward with steady HIGH (no PWM) for easier voltage measurement"""
    print("\n" + "="*60)
    print("FORWARD TEST - STEADY HIGH (No PWM)")
    print("="*60)
    print("\nThis test holds all signals steady for accurate voltage measurement")
    print("Measure voltage at L298N OUT1 and OUT2 (motor terminals)")
    print("Expected: Battery voltage (e.g., 9V if using 9V battery)\n")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Setup pins
    GPIO.setup(config.MOTOR_DRIVE_FORWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_BACKWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
    
    print("Setting direction for FORWARD:")
    print(f"  IN1 (GPIO {config.MOTOR_DRIVE_FORWARD}) = HIGH")
    print(f"  IN2 (GPIO {config.MOTOR_DRIVE_BACKWARD}) = LOW")
    print(f"  ENA (GPIO {config.MOTOR_DRIVE_PWM}) = HIGH")
    
    GPIO.output(config.MOTOR_DRIVE_FORWARD, True)
    GPIO.output(config.MOTOR_DRIVE_BACKWARD, False)
    GPIO.output(config.MOTOR_DRIVE_PWM, True)
    
    print("\n⚡ Motor should be ON now!")
    voltage = input("\nMeasure voltage at motor terminals (OUT1 to OUT2): ")
    
    try:
        v = float(voltage)
        print("\n" + "="*60)
        if v < 1:
            print("❌ PROBLEM: Voltage too low!")
            print("\nPossible causes:")
            print("1. ENA jumper still ON (remove it!)")
            print("2. Battery not connected to L298N +12V")
            print("3. Battery voltage too low or dead")
            print("4. No common ground between Pi and L298N")
            print("5. L298N damaged/overheated")
            print("\nDEBUGGING STEPS:")
            print("→ Measure battery voltage directly (should be 6-12V)")
            print("→ Measure voltage at L298N +12V pin (should match battery)")
            print("→ Check Pi GND is connected to L298N GND")
            print("→ Verify ENA jumper is physically removed")
        elif v < 3:
            print("⚠️  Voltage very low - partial conduction")
            print("\nLikely cause: ENA jumper still installed")
            print("→ Remove the ENA jumper on L298N")
        elif v < 0.7 * 9:  # Assuming ~9V battery
            print("⚠️  Voltage lower than expected")
            print("\nPossible causes:")
            print("→ Battery voltage low (measure battery directly)")
            print("→ High current draw (motor stalling?)")
            print("→ Voltage drop in wires (use thicker wires)")
        else:
            print("✅ Voltage looks good!")
            print("Motor should be spinning. If not:")
            print("→ Motor may be mechanically stuck")
            print("→ Motor may need more voltage")
            print("→ Check motor wires are connected properly")
    except:
        print("Invalid voltage entered")
    
    print("\n" + "="*60)
    
    # Cleanup
    GPIO.output(config.MOTOR_DRIVE_FORWARD, False)
    GPIO.output(config.MOTOR_DRIVE_BACKWARD, False)
    GPIO.output(config.MOTOR_DRIVE_PWM, False)
    GPIO.cleanup()


def test_backward_steady():
    """Test backward with steady HIGH"""
    print("\n" + "="*60)
    print("BACKWARD TEST - STEADY HIGH (No PWM)")
    print("="*60)
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(config.MOTOR_DRIVE_FORWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_BACKWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
    
    print("Setting direction for BACKWARD:")
    print(f"  IN1 (GPIO {config.MOTOR_DRIVE_FORWARD}) = LOW")
    print(f"  IN2 (GPIO {config.MOTOR_DRIVE_BACKWARD}) = HIGH")
    print(f"  ENA (GPIO {config.MOTOR_DRIVE_PWM}) = HIGH")
    
    GPIO.output(config.MOTOR_DRIVE_FORWARD, False)
    GPIO.output(config.MOTOR_DRIVE_BACKWARD, True)
    GPIO.output(config.MOTOR_DRIVE_PWM, True)
    
    print("\n⚡ Motor should be ON (reversed)!")
    voltage = input("\nMeasure voltage at motor terminals: ")
    
    GPIO.output(config.MOTOR_DRIVE_FORWARD, False)
    GPIO.output(config.MOTOR_DRIVE_BACKWARD, False)
    GPIO.output(config.MOTOR_DRIVE_PWM, False)
    GPIO.cleanup()
    
    print(f"\nMeasured: {voltage}V")


def test_with_pwm():
    """Test with actual PWM"""
    print("\n" + "="*60)
    print("PWM TEST")
    print("="*60)
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(config.MOTOR_DRIVE_FORWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_BACKWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
    
    # Set direction
    GPIO.output(config.MOTOR_DRIVE_FORWARD, True)
    GPIO.output(config.MOTOR_DRIVE_BACKWARD, False)
    
    # Initialize PWM
    pwm = GPIO.PWM(config.MOTOR_DRIVE_PWM, 1000)  # 1kHz
    pwm.start(0)
    
    print("\nTesting different PWM duty cycles:")
    for duty in [25, 50, 75, 100]:
        print(f"\n  PWM: {duty}%")
        pwm.ChangeDutyCycle(duty)
        time.sleep(2)
        resp = input("  Motor moving? (y/n): ")
        if resp.lower() != 'y' and duty == 100:
            print("  ⚠️  Motor not moving at 100% - check power/wiring!")
    
    pwm.stop()
    GPIO.output(config.MOTOR_DRIVE_FORWARD, False)
    GPIO.cleanup()


def main():
    print("\n" + "="*60)
    print("L298N DIAGNOSTIC TOOL")
    print("="*60)
    print("\nYou're getting only 0.25V at motor terminals")
    print("This tool will help find the problem")
    
    while True:
        print("\n" + "="*60)
        print("MENU")
        print("="*60)
        print("1. Show wiring checklist")
        print("2. Test GPIO outputs (measure at Pi pins)")
        print("3. Test FORWARD (steady HIGH - easiest to measure)")
        print("4. Test BACKWARD (steady HIGH)")
        print("5. Test with PWM (different duty cycles)")
        print("0. Exit")
        
        choice = input("\nChoice: ").strip()
        
        if choice == "0":
            break
        elif choice == "1":
            show_checklist()
        elif choice == "2":
            test_gpio_outputs()
        elif choice == "3":
            test_forward_steady()
        elif choice == "4":
            test_backward_steady()
        elif choice == "5":
            test_with_pwm()
        else:
            print("Invalid choice")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        GPIO.cleanup()
