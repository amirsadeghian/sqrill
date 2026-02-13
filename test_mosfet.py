#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XY-MOS MOSFET Diagnostic Tool
Helps diagnose why MOSFET isn't conducting properly
"""
import RPi.GPIO as GPIO
import time
import config


def test_gpio_voltage():
    """Test GPIO output voltage"""
    print("\n" + "="*60)
    print("GPIO VOLTAGE TEST")
    print("="*60)
    print(f"\nTesting GPIO {config.MOTOR_DRIVE_PWM}...")
    print("Measure voltage between GPIO pin and GND with multimeter")
    print()
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
    
    # Test steady HIGH
    print("Setting GPIO HIGH (steady)...")
    GPIO.output(config.MOTOR_DRIVE_PWM, True)
    input("Press Enter after measuring voltage (should be 3.3V)...")
    GPIO.output(config.MOTOR_DRIVE_PWM, False)
    
    # Test PWM at different duty cycles
    pwm = GPIO.PWM(config.MOTOR_DRIVE_PWM, 1000)
    pwm.start(0)
    
    for duty in [25, 50, 75, 100]:
        print(f"\nSetting PWM to {duty}% duty cycle...")
        print(f"  Average voltage should be: {3.3 * duty / 100:.2f}V")
        pwm.ChangeDutyCycle(duty)
        input(f"Press Enter after measuring voltage...")
    
    pwm.stop()
    GPIO.cleanup()


def test_mosfet_output():
    """Test XY-MOS output voltage to motor"""
    print("\n" + "="*60)
    print("XY-MOS OUTPUT VOLTAGE TEST")
    print("="*60)
    print("\nMeasure voltage at MOTOR terminals with multimeter")
    print("Expected: Battery voltage when ON (e.g., 9V if using 9V battery)")
    print()
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
    
    # Test steady HIGH (bypass PWM to eliminate frequency issues)
    print("Testing steady HIGH (no PWM)...")
    GPIO.output(config.MOTOR_DRIVE_PWM, True)
    input("Measure motor voltage now. Press Enter when done...")
    GPIO.output(config.MOTOR_DRIVE_PWM, False)
    
    print("\nTesting with PWM at 100%...")
    pwm = GPIO.PWM(config.MOTOR_DRIVE_PWM, 1000)  # 1kHz
    pwm.start(100)
    input("Measure motor voltage now. Press Enter when done...")
    pwm.stop()
    
    # Try different PWM frequencies
    print("\nTesting different PWM frequencies at 100% duty cycle...")
    for freq in [100, 1000, 5000, 10000]:
        print(f"\n  Frequency: {freq} Hz")
        pwm = GPIO.PWM(config.MOTOR_DRIVE_PWM, freq)
        pwm.start(100)
        input(f"  Measure voltage. Press Enter...")
        pwm.stop()
    
    GPIO.cleanup()


def show_wiring_check():
    """Show XY-MOS wiring checklist"""
    print("\n" + "="*60)
    print("XY-MOS WIRING CHECKLIST")
    print("="*60)
    
    print("\nYour XY-MOS module should have these pins:")
    print("  ┌─────────────┐")
    print("  │  XY-MOS     │")
    print("  ├─────────────┤")
    print("  │ VCC  (5V)   │ ← Should be connected to 5V? (check datasheet)")
    print("  │ GND         │ ← Connected to Pi GND + Battery GND")
    print("  │ SIG/PWM     │ ← Connected to GPIO 13")
    print("  │ OUT (+)     │ ← To motor (+)")
    print("  │ IN  (+)     │ ← From battery (+)")
    print("  └─────────────┘")
    
    print("\n⚠️  COMMON ISSUE: 3.3V GPIO vs 5V MOSFET")
    print("   Raspberry Pi GPIO outputs 3.3V")
    print("   Many MOSFETs need 4-5V gate voltage to fully turn on")
    print("   With only 3.3V, MOSFET partially conducts → low output voltage")
    
    print("\n✅ SOLUTIONS:")
    print("\n1. CHECK IF XY-MOS HAS A VCC PIN:")
    print("   - Some XY-MOS modules have a VCC pin that needs 5V")
    print("   - Connect VCC pin to Raspberry Pi 5V pin (Pin 2 or 4)")
    print("   - This provides 5V reference for proper MOSFET switching")
    
    print("\n2. USE LOGIC LEVEL SHIFTER:")
    print("   - Converts 3.3V GPIO to 5V signal")
    print("   - Connect: Pi GPIO → Shifter LV → Shifter HV → XY-MOS SIG")
    print("   - Cheap and effective (~$2)")
    
    print("\n3. USE DIFFERENT PWM PIN WITH VOLTAGE DIVIDER:")
    print("   - Some users connect GPIO to XY-MOS VCC instead of SIG")
    print("   - NOT recommended but might work as temporary hack")
    
    print("\n4. REPLACE WITH LOGIC-LEVEL MOSFET:")
    print("   - Get MOSFET module designed for 3.3V logic (IRLZ44N, etc.)")
    print("   - Or use L298N H-bridge (supports 3.3V inputs)")
    
    print("\n" + "="*60)


def main():
    print("\n" + "="*60)
    print("XY-MOS MOSFET DIAGNOSTIC TOOL")
    print("="*60)
    print("\nYou measured 0.25V at motor when expecting 4V+")
    print("This means the MOSFET is NOT turning on properly")
    print("\nLet's diagnose the problem...")
    
    while True:
        print("\n" + "="*60)
        print("DIAGNOSTIC MENU")
        print("="*60)
        print("1. Test GPIO output voltage (measure at GPIO pin)")
        print("2. Test XY-MOS output voltage (measure at motor)")
        print("3. Show wiring checklist & solutions")
        print("4. Quick test: Steady HIGH (no PWM)")
        print("0. Exit")
        
        choice = input("\nChoice: ").strip()
        
        if choice == "0":
            break
        elif choice == "1":
            test_gpio_voltage()
        elif choice == "2":
            test_mosfet_output()
        elif choice == "3":
            show_wiring_check()
        elif choice == "4":
            print("\n" + "="*60)
            print("QUICK TEST - STEADY HIGH (No PWM)")
            print("="*60)
            print("\nThis eliminates PWM frequency as the issue")
            print("Measure voltage at motor terminals")
            print()
            
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
            
            duration = float(input("Duration in seconds (e.g., 5): "))
            print(f"\nTurning ON for {duration}s...")
            print("Measure voltage NOW!")
            
            GPIO.output(config.MOTOR_DRIVE_PWM, True)
            time.sleep(duration)
            GPIO.output(config.MOTOR_DRIVE_PWM, False)
            
            voltage = input("\nWhat voltage did you measure? ")
            try:
                v = float(voltage)
                if v < 1:
                    print("\n⚠️  MOSFET NOT CONDUCTING PROPERLY")
                    print("   → GPIO voltage too low (3.3V insufficient)")
                    print("   → Solution: Add 3.3V→5V logic level shifter")
                    print("   → OR: Check if XY-MOS has VCC pin needing 5V")
                elif v < 0.8 * 9:  # Assuming ~9V battery
                    print("\n⚠️  MOSFET PARTIALLY CONDUCTING")
                    print("   → May work at low loads but not under load")
                    print("   → Still recommend logic level shifter")
                else:
                    print("\n✅ MOSFET IS CONDUCTING PROPERLY!")
                    print("   → Issue might be PWM frequency or duty cycle")
            except:
                pass
            
            GPIO.cleanup()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        GPIO.cleanup()
