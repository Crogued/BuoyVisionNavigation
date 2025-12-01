#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HSV Calibrator for Red Color Detection

This program allows real-time adjustment of HSV values for optimal
red color detection using interactive trackbars. It is a calibration
tool for the red buoy detection system.

The program shows the original camera image and the resulting mask,
allowing adjustment of the two HSV intervals needed to capture
the red color adequately (which crosses the 0/180 boundary of HSV space).

Author: [Your Name]  
Date: 21/05/2025
Version: 2.1 - Fixed compatibility with Raspberry Pi
"""

import cv2
import numpy as np
from picamera2 import Picamera2
import time


class HSVCalibrator:
    """
    Class for interactive calibration of HSV values for red color detection.
    
    This class creates an interface with trackbars that allows real-time
    adjustment of the limits of the two HSV intervals needed to correctly
    detect red color under different lighting conditions.
    """
    
    def __init__(self, resolution=(640, 480)):
        """
        Initializes the HSV calibrator.
        
        Args:
            resolution (tuple): Camera resolution (width, height).
        """
        # Camera initialization
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_preview_configuration(
            main={"format": 'XRGB8888', "size": resolution}
        ))
        
        # Window names (no special characters)
        self.window_controls = "HSV_Controls"
        self.window_original = "Original_Image"
        self.window_mask = "Red_Mask"
        
        # Current HSV interval values
        self.hsv_values = {
            # First interval (dark red - close to 0°)
            "l1_h": 0,    "l1_s": 150,  "l1_v": 100,
            "u1_h": 10,   "u1_s": 255,  "u1_v": 255,
            # Second interval (light red - close to 180°)  
            "l2_h": 160,  "l2_s": 130,  "l2_v": 0,
            "u2_h": 179,  "u2_s": 255,  "u2_v": 255
        }
        
        # Initial values (for reset)
        self.initial_values = self.hsv_values.copy()
        
        # Window state
        self.interface_created = False
        
    def empty_callback(self, value):
        """Empty callback function for trackbars."""
        pass
        
    def create_interface(self):
        """
        Creates the interface with windows and trackbars.
        Robust version for Raspberry Pi.
        """
        if self.interface_created:
            return True
            
        try:
            print("Creating calibration interface...")
            
            # Destroy existing windows (if any)
            cv2.destroyAllWindows()
            time.sleep(0.1)
            
            # Create main controls window
            cv2.namedWindow(self.window_controls, cv2.WINDOW_AUTOSIZE)
            
            # Wait for window creation
            time.sleep(0.2)
            cv2.waitKey(1)
            
            # Verify if window was created successfully
            try:
                # Try to create a test trackbar
                cv2.createTrackbar("test", self.window_controls, 0, 1, self.empty_callback)
                cv2.destroyWindow(self.window_controls)  # Clear test
                time.sleep(0.1)
            except:
                print("Error: Could not create trackbars. Using fixed values.")
                return False
            
            # Recreate main window
            cv2.namedWindow(self.window_controls, cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow(self.window_original, cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow(self.window_mask, cv2.WINDOW_AUTOSIZE)
            
            time.sleep(0.2)
            cv2.waitKey(1)
            
            # Create trackbars with simple names
            trackbars = [
                ("L1H", "l1_h", 179), ("L1S", "l1_s", 255), ("L1V", "l1_v", 255),
                ("U1H", "u1_h", 179), ("U1S", "u1_s", 255), ("U1V", "u1_v", 255),
                ("L2H", "l2_h", 179), ("L2S", "l2_s", 255), ("L2V", "l2_v", 255),
                ("U2H", "u2_h", 179), ("U2S", "u2_s", 255), ("U2V", "u2_v", 255)
            ]
            
            for track_name, value_key, maximum in trackbars:
                initial_value = self.hsv_values[value_key]
                cv2.createTrackbar(track_name, self.window_controls, 
                                 initial_value, maximum, self.empty_callback)
                time.sleep(0.01)  # Small pause between trackbars
                
            self.interface_created = True
            print("Interface created successfully!")
            return True
            
        except Exception as e:
            print(f"Error creating interface: {e}")
            print("Continuing with fixed values...")
            return False
            
    def update_trackbar_values(self):
        """
        Updates HSV values based on trackbars (if available).
        """
        if not self.interface_created:
            return
            
        try:
            # Mapping trackbar names to keys
            mapping = {
                "L1H": "l1_h", "L1S": "l1_s", "L1V": "l1_v",
                "U1H": "u1_h", "U1S": "u1_s", "U1V": "u1_v",
                "L2H": "l2_h", "L2S": "l2_s", "L2V": "l2_v",
                "U2H": "u2_h", "U2S": "u2_s", "U2V": "u2_v"
            }
            
            for track_name, key in mapping.items():
                value = cv2.getTrackbarPos(track_name, self.window_controls)
                self.hsv_values[key] = value
                
        except Exception as e:
            print(f"Warning: Error reading trackbars: {e}")
            
    def process_frame(self, frame):
        """
        Processes a frame applying the configured HSV mask.
        
        Args:
            frame: Image captured from camera.
            
        Returns:
            numpy.ndarray: Resulting binary mask.
        """
        # Update trackbar values
        self.update_trackbar_values()
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define HSV intervals based on current values
        lower1 = np.array([self.hsv_values["l1_h"], self.hsv_values["l1_s"], self.hsv_values["l1_v"]])
        upper1 = np.array([self.hsv_values["u1_h"], self.hsv_values["u1_s"], self.hsv_values["u1_v"]])
        lower2 = np.array([self.hsv_values["l2_h"], self.hsv_values["l2_s"], self.hsv_values["l2_v"]])  
        upper2 = np.array([self.hsv_values["u2_h"], self.hsv_values["u2_s"], self.hsv_values["u2_v"]])
        
        # Create masks for both intervals
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        
        # Combine masks with OR operation
        final_mask = cv2.bitwise_or(mask1, mask2)
        
        return final_mask
        
    def add_interface_info(self, frame):
        """
        Adds useful information to the original image.
        
        Args:
            frame: Image to add information to.
        """
        # Current state information
        status = "WITH TRACKBARS" if self.interface_created else "FIXED VALUES"
        
        # Add instructions to image
        instruction_text = [
            f"HSV CALIBRATOR - {status}",
            "Press 'q' to exit and get values",
            "Press 'r' to reset initial values",
            "Press 's' to show current values"
        ]
        
        if self.interface_created:
            instruction_text.append("Adjust trackbars for better detection")
        else:
            instruction_text.append("Use +/- keys to adjust values")
            
        # Show current values
        values_text = [
            f"L1: H={self.hsv_values['l1_h']:3d} S={self.hsv_values['l1_s']:3d} V={self.hsv_values['l1_v']:3d}",
            f"U1: H={self.hsv_values['u1_h']:3d} S={self.hsv_values['u1_s']:3d} V={self.hsv_values['u1_v']:3d}",
            f"L2: H={self.hsv_values['l2_h']:3d} S={self.hsv_values['l2_s']:3d} V={self.hsv_values['l2_v']:3d}",
            f"U2: H={self.hsv_values['u2_h']:3d} S={self.hsv_values['u2_s']:3d} V={self.hsv_values['u2_v']:3d}"
        ]
        
        # Draw text
        y_offset = 25
        for i, line in enumerate(instruction_text + values_text):
            y_pos = y_offset + (i * 22)
            color = (255, 255, 255) if i < len(instruction_text) else (0, 255, 255)
            
            # Black shadow for better readability
            cv2.putText(frame, line, (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            # Colored text on top
            cv2.putText(frame, line, (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                       
    def reset_initial_values(self):
        """Resets all values to initial values."""
        self.hsv_values = self.initial_values.copy()
        
        if self.interface_created:
            try:
                mapping = {
                    "L1H": "l1_h", "L1S": "l1_s", "L1V": "l1_v",
                    "U1H": "u1_h", "U1S": "u1_s", "U1V": "u1_v",
                    "L2H": "l2_h", "L2S": "l2_s", "L2V": "l2_v",
                    "U2H": "u2_h", "U2S": "u2_s", "U2V": "u2_v"
                }
                
                for track_name, key in mapping.items():
                    cv2.setTrackbarPos(track_name, self.window_controls, self.hsv_values[key])
                    
            except Exception as e:
                print(f"Error resetting trackbars: {e}")
                
        print("Values reset to initial values")
        
    def print_current_values(self):
        """Prints current HSV values."""
        v = self.hsv_values
        print("\n" + "="*50)
        print("CURRENT HSV VALUES")
        print("="*50)
        print(f"lower_red1 = np.array([{v['l1_h']}, {v['l1_s']}, {v['l1_v']}])")
        print(f"upper_red1 = np.array([{v['u1_h']}, {v['u1_s']}, {v['u1_v']}])")
        print(f"lower_red2 = np.array([{v['l2_h']}, {v['l2_s']}, {v['l2_v']}])")
        print(f"upper_red2 = np.array([{v['u2_h']}, {v['u2_s']}, {v['u2_v']}])")
        print("="*50)
        
    def adjust_values_keyboard(self, key):
        """
        Adjusts values using keyboard when trackbars are not available.
        """
        if self.interface_created:
            return  # Use trackbars if available
            
        # Implement keyboard adjustments (basic example)
        adjustment = 5
        if key == ord('1'):
            self.hsv_values['l1_h'] = max(0, self.hsv_values['l1_h'] - adjustment)
        elif key == ord('2'):
            self.hsv_values['l1_h'] = min(179, self.hsv_values['l1_h'] + adjustment)
        # Add more controls as needed
        
    def start(self):
        """Starts the camera."""
        self.camera.start()
        print("Camera initialized")
        
    def stop(self):
        """Releases resources and closes windows."""
        self.camera.close()
        cv2.destroyAllWindows()
        print("Calibration ended")
        
    def run(self):
        """
        Executes the main HSV calibration loop.
        """
        print("=== RED COLOR HSV CALIBRATOR ===")
        print("Controls:")
        print("  'q' - Exit and show calibrated values")
        print("  'r' - Reset initial values")
        print("  's' - Show current values")
        print("="*50)
        
        # Try to create interface with trackbars
        interface_ok = self.create_interface()
        
        if not interface_ok:
            print("Using fixed values mode (no trackbars)")
            # Create only basic windows
            cv2.namedWindow(self.window_original, cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow(self.window_mask, cv2.WINDOW_AUTOSIZE)
            
        try:
            while True:
                # Capture frame from camera
                frame = self.camera.capture_array()
                
                # Add information to interface
                self.add_interface_info(frame)
                
                # Process frame and create mask
                mask = self.process_frame(frame)
                
                # Show images
                cv2.imshow(self.window_original, frame)
                cv2.imshow(self.window_mask, mask)
                
                # Check pressed keys
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    # Exit and show final values
                    self.print_current_values()
                    break
                elif key == ord('r'):
                    # Reset initial values
                    self.reset_initial_values()
                elif key == ord('s'):
                    # Show current values
                    self.print_current_values()
                else:
                    # Try to adjust by keyboard if necessary
                    self.adjust_values_keyboard(key)
                    
        except KeyboardInterrupt:
            print("\nCalibration interrupted by user")
        except Exception as e:
            print(f"Error during calibration: {e}")
        finally:
            self.stop()


if __name__ == "__main__":
    # Camera resolution configuration
    RESOLUTION = (640, 480)
    
    # Create and execute calibrator
    calibrator = HSVCalibrator(resolution=RESOLUTION)
    calibrator.start()
    calibrator.run()
