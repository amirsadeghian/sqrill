"""
Camera Module
Handles camera input for the RC car
Can be extended for object detection, line following, etc.
"""
import cv2
import numpy as np
from picamera2 import Picamera2
from typing import Optional
import config
import time


class CameraModule:
    """Manages camera input and basic image processing"""
    
    def __init__(self, use_picamera: bool = True):
        """
        Initialize camera
        Args:
            use_picamera: True for Raspberry Pi Camera, False for USB camera
        """
        self.use_picamera = use_picamera
        self.camera = None
        self.is_recording = False
        
        # For motion detection
        self.previous_frame = None
        self.motion_detected = False
        self.motion_location = None
        
        if use_picamera:
            try:
                self.camera = Picamera2()
                camera_config = self.camera.create_preview_configuration(
                    main={"size": config.CAMERA_RESOLUTION, "format": "RGB888"}
                )
                self.camera.configure(camera_config)
                self.camera.start()
                time.sleep(2)  # Allow camera to warm up
                print("Pi Camera initialized")
            except Exception as e:
                print(f"Error initializing Pi Camera: {e}")
                print("Falling back to USB camera")
                self.use_picamera = False
                self._init_usb_camera()
        else:
            self._init_usb_camera()
    
    def _init_usb_camera(self):
        """Initialize USB camera"""
        try:
            self.camera = cv2.VideoCapture(0)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_RESOLUTION[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_RESOLUTION[1])
            self.camera.set(cv2.CAP_PROP_FPS, config.CAMERA_FRAMERATE)
            print("USB Camera initialized")
        except Exception as e:
            print(f"Error initializing USB camera: {e}")
            self.camera = None
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """
        Capture a single frame from camera
        Returns numpy array (BGR format for OpenCV) or None if capture fails
        """
        if self.camera is None:
            return None
        
        try:
            if self.use_picamera:
                frame = self.camera.capture_array()
                # Convert RGB to BGR for OpenCV
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = self.camera.read()
                if not ret:
                    return None
            return frame
        except Exception as e:
            print(f"Error capturing frame: {e}")
            return None
    
    def detect_obstacles_simple(self, frame: np.ndarray) -> bool:
        """
        Simple obstacle detection using edge detection
        Returns True if significant obstacles detected in center
        """
        if frame is None:
            return False
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Check center region
        h, w = edges.shape
        center_region = edges[h//3:2*h//3, w//3:2*w//3]
        
        # Count edge pixels
        edge_pixels = np.sum(center_region > 0)
        threshold = (center_region.shape[0] * center_region.shape[1]) * 0.1
        
        return edge_pixels > threshold
    
    def detect_colors(self, frame: np.ndarray, lower_hsv: tuple, upper_hsv: tuple):
        """
        Detect specific colors in frame
        Useful for following colored lines or detecting markers
        """
        if frame is None:
            return None
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask
        mask = cv2.inRange(hsv, np.array(lower_hsv), np.array(upper_hsv))
        
        return mask
    
    def detect_motion(self, frame: np.ndarray, threshold: int = None) -> tuple:
        """
        Detect motion by comparing current frame with previous frame
        Returns (motion_detected, motion_center, motion_area, debug_frame)
        
        Args:
            frame: Current video frame
            threshold: Minimum change percentage (0-100) to consider motion
        
        Returns:
            tuple: (bool, tuple(x,y) or None, int, np.ndarray)
        """
        if frame is None:
            return False, None, 0, None
        
        if threshold is None:
            threshold = config.MOTION_DETECTION_THRESHOLD
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        # If this is the first frame, store it and return
        if self.previous_frame is None:
            self.previous_frame = gray
            return False, None, 0, frame
        
        # Compute absolute difference between current and previous frame
        frame_delta = cv2.absdiff(self.previous_frame, gray)
        thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
        
        # Dilate to fill in holes
        thresh = cv2.dilate(thresh, None, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Update previous frame
        self.previous_frame = gray
        
        # Create debug frame
        debug_frame = frame.copy()
        
        motion_detected = False
        motion_center = None
        max_area = 0
        
        # Look for significant motion
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < config.MOTION_AREA_THRESHOLD:
                continue
            
            motion_detected = True
            
            if area > max_area:
                max_area = area
                
                # Get bounding box
                (x, y, w, h) = cv2.boundingRect(contour)
                
                # Calculate center
                motion_center = (x + w // 2, y + h // 2)
                
                # Draw on debug frame
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(debug_frame, motion_center, 5, (0, 0, 255), -1)
                cv2.putText(debug_frame, f"MOTION: {int(area)}", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        self.motion_detected = motion_detected
        self.motion_location = motion_center
        
        return motion_detected, motion_center, max_area, debug_frame
    
    def get_motion_direction(self, motion_center: tuple) -> str:
        """
        Determine direction of detected motion relative to camera center
        Returns: 'left', 'center', 'right'
        """
        if motion_center is None:
            return 'unknown'
        
        frame_width = config.CAMERA_RESOLUTION[0]
        x_pos = motion_center[0]
        
        # Divide frame into thirds
        if x_pos < frame_width / 3:
            return 'left'
        elif x_pos > 2 * frame_width / 3:
            return 'right'
        else:
            return 'center'
    
    def reset_motion_detection(self):
        """Reset motion detection baseline"""
        self.previous_frame = None
        self.motion_detected = False
        self.motion_location = None
    
    def get_frame_with_overlay(self, frame: np.ndarray, distances: dict) -> np.ndarray:
        """
        Add sensor distance overlay to frame for visualization
        """
        if frame is None:
            return None
        
        overlay = frame.copy()
        h, w = overlay.shape[:2]
        
        # Add distance text
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_pos = 30
        for direction, distance in distances.items():
            if distance is not None:
                color = (0, 255, 0) if distance > config.SAFE_DISTANCE else (0, 0, 255)
                text = f"{direction}: {distance:.1f}cm"
                cv2.putText(overlay, text, (10, y_pos), font, 0.6, color, 2)
                y_pos += 30
        
        return overlay
    
    def save_frame(self, frame: np.ndarray, filename: str):
        """Save current frame to file"""
        if frame is not None:
            cv2.imwrite(filename, frame)
    
    def start_recording(self, output_file: str = "output.avi"):
        """Start recording video"""
        if self.camera is None:
            return False
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(
            output_file, 
            fourcc, 
            config.CAMERA_FRAMERATE,
            config.CAMERA_RESOLUTION
        )
        self.is_recording = True
        return True
    
    def record_frame(self, frame: np.ndarray):
        """Add frame to video recording"""
        if self.is_recording and frame is not None:
            self.video_writer.write(frame)
    
    def stop_recording(self):
        """Stop recording video"""
        if self.is_recording:
            self.video_writer.release()
            self.is_recording = False
    
    def cleanup(self):
        """Release camera resources"""
        if self.is_recording:
            self.stop_recording()
        
        if self.camera is not None:
            if self.use_picamera:
                self.camera.stop()
            else:
                self.camera.release()


if __name__ == "__main__":
    """Test the camera"""
    print("Testing Camera Module...")
    print("Press 'q' to exit, 's' to save snapshot\n")
    
    camera = CameraModule(use_picamera=True)
    
    try:
        while True:
            frame = camera.capture_frame()
            
            if frame is not None:
                # Show frame
                cv2.imshow("Camera Feed", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f"snapshot_{int(time.time())}.jpg"
                    camera.save_frame(frame, filename)
                    print(f"Saved {filename}")
            else:
                print("No frame captured")
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\n\nCleaning up...")
    finally:
        cv2.destroyAllWindows()
        camera.cleanup()
        print("Done!")
