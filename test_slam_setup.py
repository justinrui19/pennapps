#!/usr/bin/env python3
"""
ORB-SLAM2 Test with Sample Images
Test the SLAM system while waiting for ESP32-CAM to come online
"""

import cv2
import numpy as np
import os
import time

def create_test_images():
    """Create sample images for testing ORB-SLAM2"""
    output_dir = "test_images"
    os.makedirs(output_dir, exist_ok=True)
    
    print("Creating test images for ORB-SLAM2...")
    
    # Create a series of images with simple features
    for i in range(20):
        # Create a blank image
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add some features (rectangles, circles)
        # Moving pattern to simulate camera motion
        offset_x = i * 10
        offset_y = int(i * 5)
        
        # Draw rectangles
        cv2.rectangle(img, (100 + offset_x, 100 + offset_y), 
                     (200 + offset_x, 200 + offset_y), (255, 255, 255), 2)
        cv2.rectangle(img, (300 + offset_x, 150 + offset_y), 
                     (400 + offset_x, 250 + offset_y), (255, 255, 255), 2)
        
        # Draw circles
        cv2.circle(img, (150 + offset_x, 300 + offset_y), 30, (255, 255, 255), 2)
        cv2.circle(img, (450 + offset_x, 280 + offset_y), 25, (255, 255, 255), 2)
        
        # Add some text features
        cv2.putText(img, f"Frame {i:03d}", (50 + offset_x, 50 + offset_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Save image
        filename = f"{output_dir}/frame_{i:06d}.jpg"
        cv2.imwrite(filename, img)
    
    # Create timestamps file
    with open(f"{output_dir}/timestamps.txt", "w") as f:
        for i in range(20):
            timestamp = i * 0.1  # 10 FPS
            f.write(f"{timestamp:.6f}\n")
    
    print(f"Created {20} test images in {output_dir}/")
    return output_dir

def test_orb_slam2():
    """Test ORB-SLAM2 with sample images"""
    
    # Create test images
    image_dir = create_test_images()
    
    print("\nTesting ORB-SLAM2 with sample images...")
    print("You can now run:")
    print(f"./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml {image_dir}/")
    
    # Also test with existing TUM dataset if available
    print("\nOr download a real dataset:")
    print("wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz")
    
    return image_dir

if __name__ == "__main__":
    print("ORB-SLAM2 Testing Setup")
    print("=" * 30)
    
    # Check if ORB-SLAM2 is built
    slam_executable = "./Examples/Monocular/mono_tum"
    if os.path.exists(slam_executable):
        print("✓ ORB-SLAM2 executable found")
        
        # Create test data
        test_dir = test_orb_slam2()
        
        print(f"\nTo test ORB-SLAM2:")
        print(f"cd /Users/krishnamansinghka/Downloads/ORB_SLAM2_MacOS-master")
        print(f"{slam_executable} Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml {test_dir}/")
        
    else:
        print("✗ ORB-SLAM2 not built. Run: ./build.sh")
    
    print("\nWhen ESP32-CAM comes back online, you can use:")
    print("- python3 esp32_discovery.py  (to find the camera)")
    print("- python3 esp32_mjpeg_capture.py  (to capture live stream)")
    print("- ./Examples/Monocular/mono_esp32cam_live  (direct C++ integration)")