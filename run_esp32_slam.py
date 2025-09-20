#!/usr/bin/env python3
"""
Direct ESP32-CAM SLAM Integration
Use the C++ mono_esp32cam_live executable for real SLAM processing
"""

import subprocess
import sys
import os
import time
import requests

def test_esp32_connection():
    """Test if ESP32-CAM is accessible"""
    print("Testing ESP32-CAM connection...")
    
    # Test base connection (port 80)
    try:
        response = requests.get("http://192.168.43.36", timeout=5)
        print(f"✓ ESP32-CAM base accessible: {response.status_code}")
    except Exception as e:
        print(f"✗ ESP32-CAM base connection failed: {e}")
        return False
    
    # Test stream connection (port 81)
    try:
        response = requests.get("http://192.168.43.36:81", timeout=5, stream=True)
        if response.status_code == 200:
            print("✓ ESP32-CAM stream accessible")
            response.close()
            return True
        else:
            print(f"✗ ESP32-CAM stream status: {response.status_code}")
    except Exception as e:
        print(f"✗ ESP32-CAM stream connection failed: {e}")
    
    return False

def configure_esp32_camera():
    """Configure ESP32-CAM settings for optimal SLAM"""
    print("Configuring ESP32-CAM for SLAM...")
    
    base_url = "http://192.168.43.36"
    
    settings = [
        ("framesize", 6, "VGA resolution (640x480)"),
        ("quality", 12, "Good quality for feature detection"),
        ("awb", 1, "Auto white balance ON"),
        ("aec", 1, "Auto exposure ON"),
        ("agc", 1, "Auto gain control ON"),
        ("hmirror", 0, "Horizontal mirror OFF"),
        ("vflip", 0, "Vertical flip OFF")
    ]
    
    for param, value, description in settings:
        try:
            url = f"{base_url}/control?var={param}&val={value}"
            response = requests.get(url, timeout=3)
            if response.status_code == 200:
                print(f"  ✓ {description}")
            else:
                print(f"  ⚠ {description} - Status: {response.status_code}")
        except Exception as e:
            print(f"  ✗ Failed to set {param}: {e}")
    
    # Wait for settings to apply
    time.sleep(2)

def run_slam_with_esp32():
    """Run ORB-SLAM2 with ESP32-CAM live feed"""
    
    # Check if SLAM executable exists
    slam_executable = "./Examples/Monocular/mono_esp32cam_live"
    if not os.path.exists(slam_executable):
        print(f"✗ SLAM executable not found: {slam_executable}")
        print("Please build it with: make mono_esp32cam_live")
        return False
    
    # Check vocabulary file
    vocab_path = "Vocabulary/ORBvoc.txt"
    if not os.path.exists(vocab_path):
        print(f"✗ Vocabulary file not found: {vocab_path}")
        return False
    
    # Check config file
    config_path = "Examples/Monocular/ESP32CAM_Live.yaml"
    if not os.path.exists(config_path):
        print(f"⚠ Using default config: Examples/Monocular/TUM1.yaml")
        config_path = "Examples/Monocular/TUM1.yaml"
    
    print("\n" + "="*50)
    print("🚀 Starting ORB-SLAM2 with ESP32-CAM Live Feed")
    print("="*50)
    print("Expected behavior:")
    print("• Camera stream window will appear")
    print("• Green feature points will be tracked")
    print("• Camera trajectory will be visualized")
    print("• Map points will be displayed in 3D")
    print("Press ESC or Ctrl+C to stop")
    print("="*50)
    
    # Run the SLAM system
    cmd = [slam_executable, vocab_path, config_path]
    
    try:
        print(f"Executing: {' '.join(cmd)}")
        print("Starting SLAM... (this may take a few seconds to initialize)")
        
        # Run SLAM in foreground so we can see output
        result = subprocess.run(cmd, cwd=".", capture_output=False)
        
        if result.returncode == 0:
            print("✓ SLAM completed successfully")
        else:
            print(f"✗ SLAM exited with code: {result.returncode}")
            
    except KeyboardInterrupt:
        print("\n⚠ SLAM interrupted by user")
    except Exception as e:
        print(f"✗ Error running SLAM: {e}")

def run_slam_with_webcam():
    """Fallback: Run SLAM with computer webcam"""
    print("\n📷 ESP32-CAM not available, trying webcam...")
    
    # Create a simple webcam SLAM using existing mono_kitti (modified for live feed)
    # This would require modifying the KITTI example for live camera input
    
    print("To use your webcam with SLAM:")
    print("1. Connect a USB camera")
    print("2. Modify Examples/Monocular/mono_kitti.cc for live input")
    print("3. Or use a recording app to create image sequences")
    
    # Alternative: suggest using recorded sequences
    print("\nAlternative: Test with recorded sequences:")
    print("./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml test_images/")

def main():
    print("ESP32-CAM Live SLAM System")
    print("="*40)
    
    # Test ESP32 connection
    if test_esp32_connection():
        print("✓ ESP32-CAM is accessible")
        
        # Configure camera
        configure_esp32_camera()
        
        # Run SLAM
        run_slam_with_esp32()
        
    else:
        print("✗ ESP32-CAM not accessible")
        print("\nTroubleshooting:")
        print("1. Make sure ESP32-CAM is powered on")
        print("2. Check WiFi connection")
        print("3. Verify IP address: http://192.168.43.36")
        print("4. Ensure port 81 is accessible")
        
        # Offer webcam alternative
        response = input("\nWould you like to try webcam instead? (y/n): ").strip().lower()
        if response == 'y':
            run_slam_with_webcam()

if __name__ == "__main__":
    main()