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
import threading
import argparse
from rescuer.config import get_default_stream_url, get_default_camera_base


def test_esp32_connection(camera_base: str):
    """Test if ESP32-CAM is accessible"""
    print("Testing ESP32-CAM connection...")
    
    # Test base connection (port 80)
    try:
        response = requests.get(camera_base, timeout=5)
        print(f"✓ ESP32-CAM base accessible: {response.status_code}")
    except Exception as e:
        print(f"✗ ESP32-CAM base connection failed: {e}")
        return False
    
    # Test stream connection (port 81)
    stream_base = camera_base.replace('http://', 'http://')  # no-op; preserve scheme
    try:
        response = requests.get(stream_base.replace(':80', '') + ":81", timeout=5, stream=True)
        if response.status_code == 200:
            print("✓ ESP32-CAM stream accessible")
            response.close()
            return True
        else:
            print(f"✗ ESP32-CAM stream status: {response.status_code}")
    except Exception as e:
        print(f"✗ ESP32-CAM stream connection failed: {e}")
    
    return False


def configure_esp32_camera(camera_base: str):
    """Configure ESP32-CAM settings for optimal SLAM"""
    print("Configuring ESP32-CAM for SLAM...")
    
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
            url = f"{camera_base}/control?var={param}&val={value}"
            response = requests.get(url, timeout=3)
            if response.status_code == 200:
                print(f"  ✓ {description}")
            else:
                print(f"  ⚠ {description} - Status: {response.status_code}")
        except Exception as e:
            print(f"  ✗ Failed to set {param}: {e}")
    
    # Wait for settings to apply
    time.sleep(2)


def run_slam_with_esp32(server_base: str, stream_url: str):
    """Run ORB-SLAM2 with ESP32-CAM live feed and publish pose/map points to dashboard if provided"""
    
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
    print("Press ESC or Ctrl+C to stop")
    print("="*50)
    
    cmd = [slam_executable, vocab_path, config_path, stream_url]

    def reader(proc):
        for line in iter(proc.stdout.readline, b''):
            try:
                text = line.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            print(text)
            if not server_base:
                continue
            if text.startswith('POSE '):
                parts = text.split()
                if len(parts) == 9:
                    # POSE t x y z qx qy qz qw
                    try:
                        t = float(parts[1])
                        pos = [float(parts[2]), float(parts[3]), float(parts[4])]
                        orient = [float(parts[5]), float(parts[6]), float(parts[7]), float(parts[8])]
                        payload = { 't': t, 'position': pos, 'orientation': orient }
                        try:
                            requests.post(f"{server_base.rstrip('/')}/api/pose", json=payload, timeout=0.3)
                        except Exception:
                            pass
                    except Exception:
                        pass
            elif text.startswith('MAPPOINTS '):
                try:
                    nums = [float(x) for x in text.split()[1:]]
                    # group into triples
                    pts = []
                    for i in range(0, len(nums)-2, 3):
                        pts.append([nums[i], nums[i+1], nums[i+2]])
                    if pts:
                        try:
                            requests.post(f"{server_base.rstrip('/')}/api/map_points", params={'replace': 'false'}, json=pts, timeout=0.5)
                        except Exception:
                            pass
                except Exception:
                    pass

    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        t = threading.Thread(target=reader, args=(proc,), daemon=True)
        t.start()
        proc.wait()
    except KeyboardInterrupt:
        print("\n⚠ SLAM interrupted by user")
    except Exception as e:
        print(f"✗ Error running SLAM: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', type=str, default='', help='Dashboard server base URL (e.g., http://localhost:8000)')
    parser.add_argument('--stream', type=str, default=get_default_stream_url(), help='ESP32 MJPEG stream URL (http://ip:81/stream)')
    parser.add_argument('--camera-base', type=str, default=get_default_camera_base(), help='ESP32 base URL for controls (http://ip)')
    args = parser.parse_args()

    print("ESP32-CAM Live SLAM System")
    print("="*40)
    
    if test_esp32_connection(args.camera_base):
        print("✓ ESP32-CAM is accessible")
        configure_esp32_camera(args.camera_base)
        run_slam_with_esp32(server_base=args.server, stream_url=args.stream)
    else:
        print("✗ ESP32-CAM not accessible")
        print("Troubleshoot connectivity and try again.")


if __name__ == "__main__":
    main()