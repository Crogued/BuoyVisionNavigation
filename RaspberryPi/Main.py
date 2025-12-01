"""
Red Buoy Detection and Tracking System

This program was developed for a Raspberry Pi 5 and is intended for real-time
detection of circular red objects (buoys) using the camera.
When a buoy is detected, its coordinates are sent to an Arduino
via serial communication (UART via RX/TX pins).

"""

import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2

class BuoyTracker:
    """
    Main class for red buoy detection and tracking.
    
    This class manages video capture, image processing,
    red object detection, and serial communication with Arduino.
    """
    
    def __init__(self, serial_port='/dev/ttyAMA0', baudrate=9600, 
                 resolution=(1280, 720), area_threshold=600, circ_threshold=(0.75, 1.0)):
        """
        Initializes the buoy tracker.
        
        Args:
            serial_port (str): Path to the UART serial port.
            baudrate (int): Baud rate for serial communication.
            resolution (tuple): Camera resolution (width, height).
            area_threshold (int): Minimum area (in pixels) to consider a contour.
            circ_threshold (tuple): Circularity interval to identify buoys (min, max).
        """
        # Camera initialization
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_preview_configuration(
            main={"format": 'XRGB8888', "size": resolution}
        ))
        
        '''Generalized (Very good with low light)'''
        # HSV intervals to detect red color (two intervals due to circular nature of HSV space)
        self.lower_red1 = np.array([0, 150, 100])     # Lower limit of first red interval
        self.upper_red1 = np.array([10, 255, 255])    # Upper limit of first red interval
        self.lower_red2 = np.array([150, 110, 0])     # Lower limit of second red interval
        self.upper_red2 = np.array([179, 255, 255])   # Upper limit of second red interval
    
        
        ''' Calibrated (Very good with good lighting)
        # HSV intervals to detect red color (two intervals due to circular nature of HSV space)
        self.lower_red1 = np.array([179, 255, 108])     # Lower limit of first red interval
        self.upper_red1 = np.array([179, 125, 154])    # Upper limit of first red interval
        self.lower_red2 = np.array([150, 175, 0])     # Lower limit of second red interval
        self.upper_red2 = np.array([179, 255, 255])   # Upper limit of second red interval
        '''
        
        # Image processing settings
        self.font = cv2.FONT_HERSHEY_SIMPLEX        # Font type for text
        self.area_threshold = area_threshold        # Minimum area to consider an object
        self.circularity_threshold = circ_threshold # Circularity interval for buoys
        
        # Serial connection with Arduino
        self.arduino_serial = None
        self.serial_port = serial_port
        self.baudrate = baudrate
        self._start_serial_connection()
        
    def _start_serial_connection(self):
        """Establishes serial connection with Arduino."""
        try:
            self.arduino_serial = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            # Wait for Arduino initialization
            time.sleep(2)
            print(f"Serial connection established at {self.serial_port} at {self.baudrate} bps")
        except serial.SerialException as e:
            print(f"Error opening serial port {self.serial_port}: {e}")
            self.arduino_serial = None
            
    def start(self):
        """Starts the camera and prepares system for tracking."""
        self.camera.start()
        print("Camera initialized successfully")
        
    def stop(self):
        """Releases resources and closes connections."""
        self.camera.close()
        cv2.destroyAllWindows()
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()
            print("Serial connection closed")
        print("System closed successfully")
        
    def calculate_circularity(self, contour):
        """
        Calculates the circularity of a contour.
        
        Circularity is a measure of how close a contour is to a perfect circle.
        A perfect circle has circularity = 1.0.
        
        Args:
            contour: The contour to calculate circularity for.
            
        Returns:
            float: Circularity value (between 0 and 1).
        """
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        
        # Avoid division by zero
        if perimeter == 0 or area == 0:
            return 0
            
        # Circularity formula: 4π × (area/perimeter²)
        circularity = 4 * np.pi * (area / (perimeter ** 2))
        return circularity
        
    def detect_objects(self, frame):
        """
        Detects red objects in the camera frame.
        
        Args:
            frame: Image captured from camera.
            
        Returns:
            list: List of dictionaries containing information about detected objects.
        """
        # Convert image to HSV color space (Hue, Saturation, Value)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create masks for red color detection (in two intervals)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)  # Combine both masks
        
        # Morphological operations to remove noise and improve detection
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Opening: noise removal
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Closing: fill gaps
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            # Ignore very small contours (likely noise)
            if area < self.area_threshold:
                continue
                
            # Calculate centroid using moments
            M = cv2.moments(contour)
            if M["m00"] == 0:  # Avoid division by zero
                continue
                
            cx = int(M["m10"] / M["m00"])  # Centroid X coordinate
            cy = int(M["m01"] / M["m00"])  # Centroid Y coordinate
            
            # Calculate circularity to determine if it is a buoy
            circ = self.calculate_circularity(contour)
            
            # Add object to list
            objects.append({
                "name": f"Object {i+1}",
                "centroid": (cx, cy),
                "circularity": circ,
                "contour": contour,
                "area": area
            })
            
        return objects
        
    def draw_objects(self, frame, objects):
        """
        Draws contours, centroids, and object information on the image.
        
        Args:
            frame: Camera frame to draw on.
            objects: List of detected objects.
        """
        for obj in objects:
            contour = obj["contour"]
            cx, cy = obj["centroid"]
            circ = obj["circularity"]
            name = obj["name"]
            area = obj["area"]
            
            # Prepare informative text
            text = f"{name}: ({cx},{cy}) C={circ:.2f} A={area:.0f}"
            
            # Draw contour in green
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            
            # Mark centroid with a blue circle
            cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
            
            # Add text with shadow for better readability
            cv2.putText(frame, text, (cx + 10, cy),
                        self.font, 0.8, (0, 0, 0), 6, cv2.LINE_AA)  # Black shadow
            cv2.putText(frame, text, (cx + 10, cy),
                        self.font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)  # White text
                        
    def send_serial(self, objects):
        """
        Sends coordinates of circular objects to Arduino via serial.
        
        Args:
            objects: List of detected objects.
        """
        for obj in objects:
            circ = obj["circularity"]
            min_circ, max_circ = self.circularity_threshold
            
            # Check if object has circularity within buoy range
            if min_circ <= circ <= max_circ:
                cx, cy = obj["centroid"]
                message = f"{cx},{cy}\n"
                
                print(f"Sending: {message.strip()}")
                
                # Send data if connection is available
                if self.arduino_serial and self.arduino_serial.is_open:
                    try:
                        self.arduino_serial.write(message.encode())
                        time.sleep(0.1)
                    except Exception as e:
                        print(f"Error sending data: {e}")
                        
    def run(self):
        """
        Main method to execute the real-time tracking loop.
        """
        print("Starting tracking. Press 'q' to exit.")
        
        # Create and configure view window
        window_name = "Red Buoy Detection"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 540)
        
        try:
            while True:
                # Capture camera frame
                frame = self.camera.capture_array()
                
                # Process image and detect objects
                objects = self.detect_objects(frame)
                
                # Visualize results
                self.draw_objects(frame, objects)
                
                # Send coordinates to Arduino
                self.send_serial(objects)
                
                # Show processed image
                cv2.imshow(window_name, frame)
                
                # Check if 'q' key was pressed to exit
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"Error during execution: {e}")
        finally:
            self.stop()

if __name__ == "__main__":
    # Configuration parameters
    CONFIGURATION = {
        'serial_port': '/dev/ttyAMA0',  # Raspberry Pi 5 UART port
        'baudrate': 9600,                # Communication rate with Arduino
        'resolution': (1280, 720),        # Camera resolution
        'area_threshold': 600,              # Minimum area of a valid object
        'circ_threshold': (0.68, 1.0)       # Minimum and maximum circularity for buoys
    }
    
    # Create and start tracker
    tracker = BuoyTracker(**CONFIGURATION)
    tracker.start()
    
    # Execute main loop
    tracker.run()
