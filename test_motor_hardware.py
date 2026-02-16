#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor Troubleshooting Tool
Diagnoses why motor buzzes but doesn't move despite 5-6V output
"""
import RPi.GPIO as GPIO
import time
import config


def test_steady_high_100percent():
    """Test with steady HIGH (100% - no PWM switching)"""
    print("\n" + "="*60)
    print("TEST 1: STEADY HIGH (No PWM - Pure DC)")
    print("="*60)
    print("\nThis bypasses PWM completely - motor gets constant voltage")
    print("If motor doesn't move here, it's definitely a hardware issue\n")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(config.MOTOR_DRIVE_FORWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_BACKWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
    
    # Set direction
    GPIO.output(config.MOTOR_DRIVE_FORWARD, True)
    GPIO.output(config.MOTOR_DRIVE_BACKWARD, False)
    
    # Set ENA to steady HIGH (no PWM)
    print("Setting ENA to steady HIGH for 5 seconds...")
    GPIO.output(config.MOTOR_DRIVE_PWM, True)
    
    time.sleep(5)
    
    response = input("\nDid motor move? (y/n): ")
    
    GPIO.output(config.MOTOR_DRIVE_PWM, False)
    GPIO.output(config.MOTOR_DRIVE_FORWARD, False)
    GPIO.cleanup()
    
    if response.lower() != 'y':
        print("\n❌ MOTOR NOT MOVING WITH STEADY DC")
        print("\nThis proves it's NOT a code/PWM issue!")
        print("Problem is hardware - see solutions below")
        return False
    else:
        print("\n✅ Motor moves with steady DC")
        print("Might be PWM frequency issue - test that next")
        return True


def test_different_pwm_frequencies():
    """Test various PWM frequencies"""
    print("\n" + "="*60)
    print("TEST 2: Different PWM Frequencies")
    print("="*60)
    print("\nSome motors work better at certain frequencies")
    print("Testing: 100Hz, 500Hz, 1000Hz, 5000Hz, 20000Hz\n")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(config.MOTOR_DRIVE_FORWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_BACKWARD, GPIO.OUT)
    GPIO.setup(config.MOTOR_DRIVE_PWM, GPIO.OUT)
    
    GPIO.output(config.MOTOR_DRIVE_FORWARD, True)
    GPIO.output(config.MOTOR_DRIVE_BACKWARD, False)
    
    frequencies = [100, 500, 1000, 5000, 20000]
    
    for freq in frequencies:
        print(f"\nTesting {freq} Hz at 100% duty cycle...")
        pwm = GPIO.PWM(config.MOTOR_DRIVE_PWM, freq)
        pwm.start(100)
        time.sleep(3)
        
        response = input(f"  Motor moving well at {freq}Hz? (y/n): ")
        pwm.stop()
        
        if response.lower() == 'y':
            print(f"  ✅ {freq}Hz works!")
            print(f"\n  To use this frequency, update motor_control.py line 34:")
            print(f"  self.pwm_drive = GPIO.PWM(config.MOTOR_DRIVE_PWM, {freq})")
        else:
            print(f"  ❌ {freq}Hz doesn't work")
        
        time.sleep(0.5)
    
    GPIO.output(config.MOTOR_DRIVE_FORWARD, False)
    GPIO.cleanup()


def show_diagnosis():
    """Show diagnosis and solutions"""
    print("\n" + "="*60)
    print("DIAGNOSIS: Motor Buzzes But Doesn't Move")
    print("="*60)
    
    print("\nYou measured: 5-6V at motor terminals")
    print("Motor is buzzing = receiving power but not enough to move")
    
    print("\n" + "="*60)
    print("ROOT CAUSES:")
    print("="*60)
    
    print("\n1. ⚡ VOLTAGE TOO LOW (Most likely!)")
    print("   Problem:")
    print("   - Motor needs 6-9V to start moving")
    print("   - You're only getting 5-6V")
    print("   - L298N drops 1-2V from input voltage")
    print("   - Example: 7V battery → 5-6V at motor")
    print("\n   Solutions:")
    print("   ✓ Use higher voltage battery (9-12V instead of 7-8V)")
    print("   ✓ Fresher/stronger battery (old batteries sag under load)")
    print("   ✓ Thicker wires to reduce voltage drop")
    print("   ✓ Replace L298N with more efficient driver (TB6612 only drops 0.3V)")
    
    print("\n2. 🔋 BATTERY CAN'T SUPPLY ENOUGH CURRENT")
    print("   Problem:")
    print("   - Motor needs 1-2A to start")
    print("   - Weak battery voltage collapses under load")
    print("\n   Solutions:")
    print("   ✓ Use battery with higher capacity (2000mAh+)")
    print("   ✓ Check battery C-rating (20C+ for LiPo)")
    print("   ✓ Measure battery voltage WHILE motor is on")
    print("   ✓ Parallel batteries for more current")
    
    print("\n3. ⚙️  MOTOR MECHANICALLY STUCK/WRONG MOTOR")
    print("   Problem:")
    print("   - Motor shaft stuck or gearbox jammed")
    print("   - Motor rated for higher voltage (12V motor on 6V supply)")
    print("   - Motor too powerful for this voltage")
    print("\n   Solutions:")
    print("   ✓ Test motor DIRECTLY on battery (bypass L298N)")
    print("   ✓ Check motor spec - use motor rated for your voltage")
    print("   ✓ Free up gearbox/transmission")
    print("   ✓ Try smaller/different motor")
    
    print("\n4. 🔌 L298N VOLTAGE DROP (1-2V)")
    print("   Problem:")
    print("   - L298N is inefficient, drops 1-2V")
    print("   - 7V input → 5-6V output")
    print("\n   Solutions:")
    print("   ✓ Upgrade to TB6612FNG (only 0.3V drop)")
    print("   ✓ Use higher input voltage to compensate")
    
    print("\n" + "="*60)
    print("IMMEDIATE TESTS:")
    print("="*60)
    
    print("\n1. MEASURE BATTERY VOLTAGE UNDER LOAD:")
    print("   - Measure battery with motor connected and running")
    print("   - If it drops below 6V → weak battery")
    print("   - Should stay above 7V for good performance")
    
    print("\n2. TEST MOTOR DIRECTLY ON BATTERY:")
    print("   - Disconnect motor from L298N")
    print("   - Connect motor wires directly to battery")
    print("   - Does it spin? → Motor is fine, need better power")
    print("   - Still buzzes? → Motor needs higher voltage or is damaged")
    
    print("\n3. TRY HIGHER VOLTAGE BATTERY:")
    print("   - If using 2S LiPo (7.4V) → Try 3S (11.1V)")
    print("   - If using 6x AA (7.2V) → Try 8x AA (9.6V)")
    print("   - L298N works up to 12V")
    
    print("\n" + "="*60)


def main():
    print("\n" + "="*60)
    print("MOTOR TROUBLESHOOTING TOOL")
    print("="*60)
    print("\nMotor is buzzing but not moving despite 5-6V")
    print("Let's find out if it's code or hardware")
    
    while True:
        print("\n" + "="*60)
        print("MENU")
        print("="*60)
        print("1. Test with steady HIGH (no PWM) - PROVES if code issue")
        print("2. Test different PWM frequencies")
        print("3. Show diagnosis & solutions")
        print("4. Quick voltage test guide")
        print("0. Exit")
        
        choice = input("\nChoice: ").strip()
        
        if choice == "0":
            break
        elif choice == "1":
            if not test_steady_high_100percent():
                show_diagnosis()
        elif choice == "2":
            test_different_pwm_frequencies()
        elif choice == "3":
            show_diagnosis()
        elif choice == "4":
            print("\n" + "="*60)
            print("VOLTAGE MEASUREMENT GUIDE")
            print("="*60)
            print("\nMeasure these voltages WITH MULTIMETER:")
            print("\n1. Battery voltage (no load): ___V")
            print("   Should be: 7-12V for good performance")
            print("\n2. Battery voltage WHILE motor runs: ___V")
            print("   Should NOT drop more than 1V")
            print("   If drops >1V → battery too weak")
            print("\n3. L298N +12V pin: ___V")
            print("   Should match battery voltage")
            print("\n4. Motor output (OUT1 to OUT2) while running: ___V")
            print("   Currently: 5-6V (too low!)")
            print("   Should be: >7V for reliable operation")
            print("\n5. Calculate L298N voltage drop:")
            print("   Drop = Battery voltage - Motor output voltage")
            print("   L298N typically drops 1-2V")
            print("   If drop >2V → L298N problem or wiring resistance")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        GPIO.cleanup()
