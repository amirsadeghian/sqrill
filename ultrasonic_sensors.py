"""
Ultrasonic Sensor Module
Handles reading from HC-SR04 ultrasonic sensors
"""
import RPi.GPIO as GPIO
import time
from typing import Dict, Optional
import config


class UltrasonicSensor:
    """Class to handle a single ultrasonic sensor"""
    
    def __init__(self, trigger_pin: int, echo_pin: int, name: str = "Sensor"):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.name = name
        
        # Setup GPIO pins
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trigger_pin, False)
        
    def get_distance(self) -> Optional[float]:
        """
        Measure distance using ultrasonic sensor
        Returns distance in centimeters, or None if measurement fails
        """
        try:
            # Send trigger pulse
            GPIO.output(self.trigger_pin, True)
            time.sleep(0.00001)  # 10 microseconds
            GPIO.output(self.trigger_pin, False)
            
            # Wait for echo to start
            timeout_start = time.time()
            pulse_start = time.time()
            while GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start - timeout_start > config.SENSOR_TIMEOUT:
                    return None
            
            # Wait for echo to end
            pulse_end = time.time()
            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end - pulse_start > config.SENSOR_TIMEOUT:
                    return None
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound: 34300 cm/s / 2
            distance = round(distance, 2)
            
            # Validate reading
            if distance > config.MAX_DISTANCE or distance < 2:
                return None
                
            return distance
            
        except Exception as e:
            print(f"Error reading {self.name}: {e}")
            return None
    
    def cleanup(self):
        """Clean up GPIO pins"""
        GPIO.cleanup([self.trigger_pin, self.echo_pin])


class SensorArray:
    """Manages multiple ultrasonic sensors"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Initialize sensors
        self.sensors = {
            'left': UltrasonicSensor(
                config.SENSOR_LEFT['trigger'],
                config.SENSOR_LEFT['echo'],
                "Left Sensor"
            ),
            'center': UltrasonicSensor(
                config.SENSOR_CENTER['trigger'],
                config.SENSOR_CENTER['echo'],
                "Center Sensor"
            ),
            'right': UltrasonicSensor(
                config.SENSOR_RIGHT['trigger'],
                config.SENSOR_RIGHT['echo'],
                "Right Sensor"
            )
        }
        
        # Allow sensors to settle
        time.sleep(0.5)
    
    def read_all(self) -> Dict[str, Optional[float]]:
        """
        Read distances from all sensors
        Returns dictionary with sensor names as keys and distances as values
        """
        distances = {}
        for name, sensor in self.sensors.items():
            distances[name] = sensor.get_distance()
            time.sleep(0.05)  # Small delay between readings
        return distances
    
    def get_safe_distances(self) -> Dict[str, float]:
        """
        Get distances from all sensors, replacing None with max distance
        """
        distances = self.read_all()
        return {
            name: (dist if dist is not None else config.MAX_DISTANCE)
            for name, dist in distances.items()
        }
    
    def is_path_clear(self, direction: str = 'center') -> bool:
        """
        Check if path is clear in a specific direction
        """
        distance = self.sensors[direction].get_distance()
        if distance is None:
            return True  # Assume clear if no reading
        return distance > config.SAFE_DISTANCE
    
    def get_closest_obstacle(self) -> tuple[str, float]:
        """
        Find the closest obstacle from all sensors
        Returns tuple of (direction, distance)
        """
        distances = self.get_safe_distances()
        closest_direction = min(distances, key=distances.get)
        return closest_direction, distances[closest_direction]
    
    def cleanup(self):
        """Clean up all sensors"""
        for sensor in self.sensors.values():
            sensor.cleanup()
        GPIO.cleanup()


if __name__ == "__main__":
    """Test the sensors"""
    print("Testing Ultrasonic Sensors...")
    print("Press Ctrl+C to exit\n")
    
    sensors = SensorArray()
    
    try:
        while True:
            distances = sensors.read_all()
            print(f"\rLeft: {distances['left']:>6.1f}cm | "
                  f"Center: {distances['center']:>6.1f}cm | "
                  f"Right: {distances['right']:>6.1f}cm", end='')
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\nCleaning up...")
        sensors.cleanup()
        print("Done!")
