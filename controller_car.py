#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PS Controller Car Control
Direct control of RC car using PlayStation controller
"""
import sys
import signal

try:
    from ps_controller import PSControllerCarInterface
    from motor_control import MotorController
except ImportError as e:
    print("=" * 60)
    print("❌ IMPORT ERROR")
    print("=" * 60)
    print(f"\nError: {e}")
    print("\nMissing dependency. Install with:")
    print("   sudo apt-get install python3-evdev")
    print("   pip install evdev")
    print("\nOr install all requirements:")
    print("   pip install -r requirements.txt")
    print("=" * 60)
    sys.exit(1)


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\n⚠️ Interrupt received, shutting down...")
    sys.exit(0)


if __name__ == "__main__":
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 60)
    print("🎮 PS CONTROLLER RC CAR CONTROL")
    print("=" * 60)
    print()
    
    # Initialize motors
    print("Initializing motors...")
    motors = MotorController()
    
    # Create controller interface
    print("Connecting to PS controller...")
    car = PSControllerCarInterface(motors)
    
    if not car.controller.device:
        print("\n" + "=" * 60)
        print("❌ CONTROLLER NOT FOUND")
        print("=" * 60)
        print("\nTroubleshooting:")
        print("1. Plug in the USB adapter")
        print("2. Connect PS controller to adapter")
        print("3. Check adapter is recognized:")
        print("   lsusb | grep 0810")
        print("4. Check device permissions:")
        print("   ls -l /dev/input/")
        print("5. Add user to input group:")
        print("   sudo usermod -a -G input $USER")
        print("   (then logout and login)")
        print("\nRun test mode:")
        print("   python ps_controller.py")
        print("=" * 60)
        motors.cleanup()
        sys.exit(1)
    
    # Run controller
    try:
        car.run()
    finally:
        motors.cleanup()
        print("Goodbye! 🎮")
