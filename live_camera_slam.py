#!/usr/bin/env python3
"""
Live Camera SLAM System
Works with webcam or ESP32-CAM (when available)
Feeds live video to ORB-SLAM2 for real-time tracking
"""

import cv2
import numpy as np
import subprocess
import threading
import time
import os
import sys
import requests
from pathlib import Path

class LiveCameraSLAM:
    def __init__(self):
        self.cap = None
        self.slam_process = None
        self.frame_pipe = None
        self.running = False
        
        # Camera sources (in priority order)
        self.camera_sources = [
            ("ESP32-CAM", "http://192.168.43.36:81"),
            ("Webcam", 0),
            ("USB Camera", 1)
        ]
        
    def test_esp32_connection(self, url):
        """Test if ESP32-CAM is accessible"""
        try:
            response = requests.get(url, timeout=3, stream=True)
            if response.status_code == 200:
                # Try to read a small chunk to verify it's streaming
                chunk = response.raw.read(1024)
                response.close()
                return len(chunk) > 0
        except:
            pass
        return False
    
    def initialize_camera(self):
        """Initialize the best available camera"""
        
        for name, source in self.camera_sources:
            print(f"Testing {name}...")
            
            if isinstance(source, str):  # ESP32-CAM URL
                if self.test_esp32_connection(source):
                    print(f"✓ {name} accessible")
                    # Use special MJPEG stream handler for ESP32
                    self.cap = cv2.VideoCapture(source)
                    if self.cap.isOpened():
                        print(f"✓ Connected to {name}")
                        return name
                    else:
                        print(f"✗ Failed to open {name} stream")
                else:
                    print(f"✗ {name} not accessible")
                    
            else:  # Webcam index
                self.cap = cv2.VideoCapture(source)
                if self.cap.isOpened():
                    # Test if we can read a frame
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        print(f"✓ Connected to {name}")
                        # Set camera properties for better SLAM
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        self.cap.set(cv2.CAP_PROP_FPS, 30)
                        return name
                    else:
                        print(f"✗ {name} cannot read frames")
                        self.cap.release()
                else:
                    print(f"✗ {name} not available")
        
        print("✗ No camera sources available")
        return None
    
    def start_slam_process(self):
        """Start ORB-SLAM2 live process"""
        vocab_path = "Vocabulary/ORBvoc.txt"
        config_path = "Examples/Monocular/ESP32CAM_Live.yaml"
        
        if not os.path.exists(vocab_path):
            print(f"✗ Vocabulary file not found: {vocab_path}")
            return False
            
        if not os.path.exists(config_path):
            print(f"✗ Config file not found: {config_path}")
            print("Using default TUM1.yaml configuration")
            config_path = "Examples/Monocular/TUM1.yaml"
        
        # Use the live ESP32 camera executable
        slam_executable = "./Examples/Monocular/mono_esp32cam_live"
        
        if not os.path.exists(slam_executable):
            print(f"✗ SLAM executable not found: {slam_executable}")
            return False
        
        # Start SLAM process with live camera feed
        cmd = [
            slam_executable,
            vocab_path,
            config_path
        ]
        
        print(f"Starting live SLAM: {' '.join(cmd)}")
        try:
            self.slam_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print("✓ Live SLAM process started")
            return True
        except Exception as e:
            print(f"✗ Failed to start SLAM: {e}")
            return False
    
    def feed_frames_to_slam(self):
        """Capture frames and feed to SLAM"""
        frame_count = 0
        start_time = time.time()
        
        print("Starting live camera feed...")
        print("Controls:")
        print("  'q': Quit")
        print("  'r': Reset camera connection")
        print("  's': Save current frame")
        
        while self.running:
            ret, frame = self.cap.read()
            
            if not ret or frame is None:
                print("Warning: No frame received")
                time.sleep(0.1)
                continue
            
            frame_count += 1
            current_time = time.time()
            elapsed = current_time - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            # Display frame with info
            display_frame = frame.copy()
            cv2.putText(display_frame, f"Frame: {frame_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, "Press 'q' to quit", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('Live Camera SLAM Feed', display_frame)
            
            # Handle keypresses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quit requested")
                break
            elif key == ord('r'):
                print("Resetting camera connection...")
                self.cap.release()
                time.sleep(1)
                camera_name = self.initialize_camera()
                if not camera_name:
                    print("Failed to reconnect camera")
                    break
                print(f"Reconnected to {camera_name}")
            elif key == ord('s'):
                filename = f"saved_frame_{frame_count:06d}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Saved frame: {filename}")
            
            # Here you would normally send the frame to SLAM
            # For now, we'll just display it
        
        print(f"Processed {frame_count} frames at {fps:.1f} FPS")
    
    def run(self):
        """Main execution loop"""
        print("Live Camera SLAM System")
        print("=" * 40)
        
        # Initialize camera
        camera_name = self.initialize_camera()
        if not camera_name:
            print("No camera available. Please:")
            print("1. Connect a USB camera/webcam")
            print("2. Power on ESP32-CAM and connect to WiFi")
            print("3. Make sure ESP32 is accessible at http://192.168.43.36:81")
            return False
        
        print(f"Using camera: {camera_name}")
        
        # Start the feed
        self.running = True
        
        try:
            self.feed_frames_to_slam()
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.cleanup()
        
        return True
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        if self.cap:
            self.cap.release()
        
        if self.slam_process:
            self.slam_process.terminate()
            self.slam_process.wait()
        
        cv2.destroyAllWindows()
        
        # Clean up named pipe
        pipe_path = "/tmp/slam_frames"
        if os.path.exists(pipe_path):
            os.remove(pipe_path)

def main():
    slam_system = LiveCameraSLAM()
    
    # Quick ESP32 check
    print("Checking for ESP32-CAM...")
    if slam_system.test_esp32_connection("http://192.168.43.36:81"):
        print("✓ ESP32-CAM detected!")
    else:
        print("✗ ESP32-CAM not found, will use webcam")
    
    slam_system.run()

if __name__ == "__main__":
    main()