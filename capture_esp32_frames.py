#!/usr/bin/env python3

"""
ESP32-CAM ORB-SLAM2 Bridge
This script captures frames from ESP32-CAM and feeds them to ORB-SLAM2 via named pipes
Based on your working ESP32 camera code
"""

import cv2
import numpy as np
import requests
import time
import os
import sys
from pathlib import Path

# ESP32 URL
URL = "http://192.168.43.36"

def set_resolution(url: str, index: int=6, verbose: bool=False):
    """Set ESP32-CAM resolution - default to VGA (640x480) for SLAM"""
    try:
        if verbose:
            resolutions = "10: UXGA(1600x1200)\n9: SXGA(1280x1024)\n8: XGA(1024x768)\n7: SVGA(800x600)\n6: VGA(640x480)\n5: CIF(400x296)\n4: QVGA(320x240)\n3: HQVGA(240x176)\n0: QQVGA(160x120)"
            print("Available resolutions:\n{}".format(resolutions))

        if index in [10, 9, 8, 7, 6, 5, 4, 3, 0]:
            requests.get(url + "/control?var=framesize&val={}".format(index))
            print(f"Resolution set to index {index}")
        else:
            print("Wrong index")
    except Exception as e:
        print(f"SET_RESOLUTION: {e}")

def set_quality(url: str, value: int=15, verbose: bool=False):
    """Set image quality (10-63, lower = better quality)"""
    try:
        if value >= 10 and value <= 63:
            requests.get(url + "/control?var=quality&val={}".format(value))
            print(f"Quality set to {value}")
    except Exception as e:
        print(f"SET_QUALITY: {e}")

def set_awb(url: str, awb: bool=True):
    """Set auto white balance"""
    try:
        requests.get(url + "/control?var=awb&val={}".format(1 if awb else 0))
        print(f"Auto white balance: {awb}")
    except Exception as e:
        print(f"SET_AWB: {e}")

def save_frames_for_slam(output_dir="captured_frames"):
    """Capture frames from ESP32-CAM for offline SLAM processing"""
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Configure camera settings
    set_resolution(URL, index=6)  # VGA resolution
    set_quality(URL, value=15)    # Good quality
    set_awb(URL, awb=True)       # Enable auto white balance
    
    # Use requests-based streaming instead of VideoCapture
    stream_url = URL
    print(f"Connecting to stream: {stream_url}")
    
    try:
        response = requests.get(stream_url, stream=True, timeout=10)
        if response.status_code != 200:
            print(f"ERROR: Stream returned status code {response.status_code}")
            return False
    except Exception as e:
        print(f"ERROR: Cannot connect to stream: {e}")
        return False

if __name__ == "__main__":
    print("ESP32-CAM Frame Capture for ORB-SLAM2")
    print("=" * 50)
    
    # Test camera connection and try to initialize stream
    try:
        print("Testing ESP32-CAM base connection...")
        response = requests.get(URL, timeout=5)
        print(f"✓ ESP32-CAM base accessible: {response.status_code}")
        
        # Try to configure camera first
        print("Configuring camera settings...")
        try:
            set_resolution(URL, index=6)  # VGA resolution
            set_quality(URL, value=15)    # Good quality
            set_awb(URL, awb=True)       # Enable auto white balance
            print("✓ Camera configured")
        except Exception as config_error:
            print(f"⚠ Camera configuration failed: {config_error}")
        
        # Now test stream connection
        print("Testing stream connection...")
        response = requests.get(URL + ":81", stream=True, timeout=10)
        if response.status_code == 200:
            print("✓ Camera stream accessible")
        else:
            print(f"✗ Camera stream returned: {response.status_code}")
            sys.exit(1)
            
    except Exception as e:
        print(f"✗ Cannot connect to camera: {e}")
        print("Make sure ESP32-CAM is powered on and connected to network")
        sys.exit(1)
    
    # Start frame capture
    save_frames_for_slam()