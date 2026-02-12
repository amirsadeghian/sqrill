#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
System Check Script
Verifies that your Raspberry Pi meets all requirements
"""
import sys
import platform

def check_python_version():
    """Check Python version"""
    print("=" * 60)
    print("🐍 PYTHON VERSION CHECK")
    print("=" * 60)
    
    version = sys.version_info
    version_str = f"{version.major}.{version.minor}.{version.micro}"
    
    print(f"Current Python: {version_str}")
    print(f"Platform: {platform.platform()}")
    print(f"Architecture: {platform.machine()}")
    
    if version.major < 3 or (version.major == 3 and version.minor < 8):
        print("\n❌ ERROR: Python 3.8 or newer required!")
        print(f"   You have: Python {version_str}")
        print("\nTo upgrade Python:")
        print("   sudo apt-get update")
        print("   sudo apt-get install python3.9")
        return False
    else:
        print(f"\n✅ Python version OK (>= 3.8)")
        return True

def check_dependencies():
    """Check if dependencies are installed"""
    print("\n" + "=" * 60)
    print("📦 DEPENDENCY CHECK")
    print("=" * 60)
    
    dependencies = {
        'RPi.GPIO': 'RPi.GPIO',
        'cv2': 'opencv-python',
        'numpy': 'numpy',
        'flask': 'Flask',
        'socketio': 'flask-socketio',
        'evdev': 'evdev',
    }
    
    missing = []
    
    for module, package in dependencies.items():
        try:
            __import__(module)
            print(f"✅ {package}")
        except ImportError:
            print(f"❌ {package} - NOT INSTALLED")
            missing.append(package)
    
    if missing:
        print(f"\n⚠️  Missing {len(missing)} package(s)")
        print("\nInstall with:")
        print("   pip install -r requirements.txt")
        return False
    else:
        print("\n✅ All dependencies installed")
        return True

def check_gpio_permissions():
    """Check GPIO permissions"""
    print("\n" + "=" * 60)
    print("🔌 GPIO PERMISSIONS CHECK")
    print("=" * 60)
    
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        print("✅ GPIO access OK")
        GPIO.cleanup()
        return True
    except Exception as e:
        print(f"❌ GPIO access failed: {e}")
        print("\nFix with:")
        print("   sudo usermod -a -G gpio $USER")
        print("   (then logout and login)")
        return False

def check_camera():
    """Check if camera is available"""
    print("\n" + "=" * 60)
    print("📷 CAMERA CHECK")
    print("=" * 60)
    
    try:
        from picamera2 import Picamera2
        camera = Picamera2()
        camera.close()
        print("✅ Pi Camera detected")
        return True
    except Exception as e:
        print(f"⚠️  Pi Camera not available: {e}")
        print("   (Optional - car works without camera)")
        return True  # Not critical

def check_controller():
    """Check if PS controller is connected"""
    print("\n" + "=" * 60)
    print("🎮 CONTROLLER CHECK")
    print("=" * 60)
    
    try:
        from evdev import list_devices, InputDevice
        devices = [InputDevice(path) for path in list_devices()]
        
        found_controller = False
        for device in devices:
            if "dual" in device.name.lower() or "psx" in device.name.lower():
                print(f"✅ Controller found: {device.name}")
                found_controller = True
        
        if not found_controller:
            print("⚠️  PS Controller not detected")
            print("   (Optional - works with web/keyboard control)")
        
        return True  # Not critical
    except Exception as e:
        print(f"⚠️  Controller check failed: {e}")
        print("   (Optional feature)")
        return True

def main():
    """Run all checks"""
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║" + " " * 10 + "SQRILL RC CAR - SYSTEM CHECK" + " " * 20 + "║")
    print("╚" + "═" * 58 + "╝")
    print()
    
    checks = [
        ("Python Version", check_python_version),
        ("Dependencies", check_dependencies),
        ("GPIO Access", check_gpio_permissions),
        ("Camera", check_camera),
        ("Controller", check_controller),
    ]
    
    results = []
    for name, check_func in checks:
        try:
            result = check_func()
            results.append((name, result))
        except Exception as e:
            print(f"\n❌ {name} check failed: {e}")
            results.append((name, False))
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 SUMMARY")
    print("=" * 60)
    
    critical_passed = all(result for name, result in results[:3])  # First 3 are critical
    optional_passed = all(result for name, result in results[3:])
    
    if critical_passed:
        print("✅ All critical checks passed!")
        print("🚗 Your RC car is ready to run!")
        
        if not optional_passed:
            print("\n⚠️  Some optional features unavailable")
            print("   Car will work but with limited functionality")
    else:
        print("❌ Some critical checks failed")
        print("⚠️  Please fix the issues above before running")
    
    print("\n" + "=" * 60)
    print()
    
    return 0 if critical_passed else 1

if __name__ == "__main__":
    sys.exit(main())
