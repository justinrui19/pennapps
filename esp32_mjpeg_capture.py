#!/usr/bin/env python3
"""
ESP32-CAM MJPEG Stream Capture for ORB-SLAM2
Direct MJPEG parsing approach for reliable ESP32 camera connection
"""

import cv2
import numpy as np
import requests
import time
import os
import sys
from pathlib import Path

# ESP32-CAM Configuration
URL = "http://192.168.43.36"
STREAM_URL = "http://192.168.43.36"

def set_resolution(url: str, index: int=6, verbose: bool=False):
    """Set camera resolution (6=VGA 640x480)"""
    try:
        if verbose:
            print(f"Setting resolution to index {index}")
        requests.get(url + "/control?var=framesize&val={}".format(index), timeout=3)
        return True
    except:
        return False

def set_quality(url: str, value: int=15, verbose: bool=False):
    """Set JPEG quality (0=highest, 63=lowest)"""
    try:
        if verbose:
            print(f"Setting quality to {value}")
        requests.get(url + "/control?var=quality&val={}".format(value), timeout=3)
        return True
    except:
        return False

def set_awb(url: str, awb: bool=True):
    """Set auto white balance"""
    try:
        requests.get(url + "/control?var=awb&val={}".format(1 if awb else 0), timeout=3)
        return True
    except:
        return False

class MJPEGStreamReader:
    """MJPEG stream reader for ESP32-CAM"""
    
    def __init__(self, url):
        self.url = url
        self.response = None
        self.buffer = b''
        
    def connect(self):
        """Connect to MJPEG stream"""
        try:
            self.response = requests.get(self.url, stream=True, timeout=10)
            if self.response.status_code == 200:
                print(f"✓ Connected to MJPEG stream: {self.url}")
                return True
            else:
                print(f"✗ Stream returned status: {self.response.status_code}")
                return False
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def read_frame(self):
        """Read next frame from MJPEG stream"""
        if not self.response:
            return None
            
        try:
            # Look for JPEG start marker (FFD8)
            while True:
                chunk = self.response.raw.read(4096)
                if not chunk:
                    return None
                    
                self.buffer += chunk
                
                # Find JPEG boundaries
                start_marker = b'\xff\xd8'  # JPEG start
                end_marker = b'\xff\xd9'    # JPEG end
                
                start_idx = self.buffer.find(start_marker)
                if start_idx == -1:
                    # No start found, keep reading
                    continue
                    
                # Look for end marker after start
                search_start = start_idx + 2
                end_idx = self.buffer.find(end_marker, search_start)
                
                if end_idx != -1:
                    # Found complete JPEG
                    jpeg_data = self.buffer[start_idx:end_idx + 2]
                    self.buffer = self.buffer[end_idx + 2:]  # Remove processed data
                    
                    # Decode JPEG
                    nparr = np.frombuffer(jpeg_data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    return frame
                    
                # If buffer gets too large, trim it
                if len(self.buffer) > 100000:
                    self.buffer = self.buffer[-50000:]
                    
        except Exception as e:
            print(f"Frame read error: {e}")
            return None
    
    def close(self):
        """Close the stream"""
        if self.response:
            self.response.close()

def capture_frames_live():
    """Live frame capture with SLAM processing"""
    
    print("ESP32-CAM Live Stream Capture")
    print("Controls:")
    print("  's': Save current frame")
    print("  'q': Quit")
    print("  'r': Reset stream connection")
    
    # Configure camera
    print("Configuring ESP32-CAM...")
    set_resolution(URL, index=6)  # VGA
    set_quality(URL, value=12)    # Good quality  
    set_awb(URL, awb=True)
    time.sleep(1)  # Allow settings to apply
    
    # Initialize stream
    stream = MJPEGStreamReader(STREAM_URL)
    if not stream.connect():
        print("Failed to connect to stream")
        return False
    
    frame_count = 0
    saved_count = 0
    
    # Create output directory
    output_dir = "esp32_frames"
    os.makedirs(output_dir, exist_ok=True)
    
    try:
        while True:
            frame = stream.read_frame()
            
            if frame is None:
                print("No frame received, attempting reconnection...")
                stream.close()
                time.sleep(1)
                if not stream.connect():
                    print("Reconnection failed")
                    break
                continue
            
            frame_count += 1
            
            # Display frame info
            display_frame = frame.copy()
            cv2.putText(display_frame, f"Frames: {frame_count}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Saved: {saved_count}", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display_frame, "Press 's' to save, 'q' to quit", (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow('ESP32-CAM Live Stream', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save frame
                timestamp = time.time()
                filename = f"{output_dir}/frame_{saved_count:06d}_{timestamp:.3f}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Saved: {filename}")
                saved_count += 1
            elif key == ord('r'):
                # Reset connection
                print("Resetting stream connection...")
                stream.close()
                time.sleep(1)
                if not stream.connect():
                    print("Reconnection failed")
                    break
                    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        stream.close()
        cv2.destroyAllWindows()
    
    print(f"Captured {frame_count} frames, saved {saved_count} frames")
    return True

def test_connection():
    """Test ESP32-CAM connectivity"""
    print("Testing ESP32-CAM connection...")
    
    # Test base connection
    try:
        response = requests.get(URL, timeout=5)
        print(f"✓ Base connection: {response.status_code}")
    except Exception as e:
        print(f"✗ Base connection failed: {e}")
        return False
    
    # Test stream connection
    try:
        response = requests.get(STREAM_URL, stream=True, timeout=5)
        print(f"✓ Stream connection: {response.status_code}")
        response.close()
    except Exception as e:
        print(f"✗ Stream connection failed: {e}")
        return False
    
    return True

if __name__ == "__main__":
    print("ESP32-CAM MJPEG Stream Capture")
    print("=" * 40)
    
    # Test connection first
    if not test_connection():
        print("Connection test failed. Make sure ESP32-CAM is accessible.")
        sys.exit(1)
    
    # Start live capture
    capture_frames_live()