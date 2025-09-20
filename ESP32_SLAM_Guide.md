# ESP32-CAM ORB-SLAM2 Usage Guide

## Quick Start

1. **Run ORB-SLAM2 with your ESP32 camera:**
```bash
cd /Users/krishnamansinghka/Downloads/ORB_SLAM2_MacOS-master
./Examples/Monocular/mono_esp32cam_live Vocabulary/ORBvoc.txt Examples/Monocular/ESP32CAM_Optimized.yaml http://192.168.43.36:81/stream
```

2. **Controls during SLAM:**
   - **ESC**: Stop SLAM and save trajectory
   - **Wait**: Let the vocabulary load (takes 1-2 minutes first time)
   - **Move camera**: Move your ESP32 camera to build the map

## Camera Settings

### For better SLAM performance:
- Use VGA resolution (640x480) - good balance of quality and performance
- Ensure good lighting
- Move the camera slowly and steadily
- Avoid rapid movements or rotations
- Include visual features (textures, corners, edges)

### ESP32-CAM resolution settings (from your Python code):
- `index=6`: VGA (640x480) - **Recommended for SLAM**
- `index=5`: CIF (400x296) - Faster but lower quality
- `index=7`: SVGA (800x600) - Higher quality but slower

## Camera Calibration

The current configuration uses approximate calibration parameters. For better accuracy:

1. **Calibrate your specific ESP32-CAM:**
   - Use OpenCV's camera calibration tools
   - Print a checkerboard pattern
   - Capture multiple images from different angles
   - Calculate intrinsic parameters (fx, fy, cx, cy) and distortion coefficients

2. **Update the YAML file** with your calibrated parameters

## Output Files

After running SLAM, you'll get:
- `CameraTrajectory.txt`: Full camera trajectory
- `KeyFrameTrajectory.txt`: Keyframe poses only

## Troubleshooting

### If SLAM doesn't start:
1. Check camera stream: `curl -I http://192.168.43.36:81/stream`
2. Verify camera resolution matches YAML settings
3. Ensure good lighting and textured environment

### If tracking is poor:
1. Reduce ORB feature count in YAML
2. Adjust FAST thresholds
3. Improve lighting conditions
4. Move camera more slowly

### If vocabulary loading is slow:
- This is normal for the first run (1-2 minutes)
- Subsequent runs will be faster

## Integration with Your Python Code

You can run both systems simultaneously:
1. **Python script**: Human detection and monitoring
2. **ORB-SLAM2**: 3D mapping and localization

Both can access the same camera stream at `http://192.168.43.36:81/stream`.

## Camera Settings via HTTP API

Using your existing ESP32 setup:
```python
# Set resolution for SLAM (VGA recommended)
requests.get("http://192.168.43.36/control?var=framesize&val=6")

# Set quality (lower = better quality, 10-63 range)
requests.get("http://192.168.43.36/control?var=quality&val=15")

# Enable auto white balance
requests.get("http://192.168.43.36/control?var=awb&val=1")
```