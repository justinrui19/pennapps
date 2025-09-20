import cv2
import numpy as np
from ultralytics import YOLO
import requests

'''
INFO SECTION
- if you want to monitor raw parameters of ESP32CAM, open the browser and go to http://192.168.x.x/status
- command can be sent through an HTTP get composed in the following way http://192.168.x.x/control?var=VARIABLE_NAME&val=VALUE (check varname and value in status)
'''

# ESP32 URL
URL = "http://192.168.43.36"
AWB = True

# Human detection and opencv setup
cap = cv2.VideoCapture(URL + ":81/stream")
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer size to minimize latency
human_detector = YOLO('yolov8n.pt')  # Load YOLOv8 model for human detection

def set_resolution(url: str, index: int=1, verbose: bool=False):
    try:
        if verbose:
            resolutions = "10: UXGA(1600x1200)\n9: SXGA(1280x1024)\n8: XGA(1024x768)\n7: SVGA(800x600)\n6: VGA(640x480)\n5: CIF(400x296)\n4: QVGA(320x240)\n3: HQVGA(240x176)\n0: QQVGA(160x120)"
            print("available resolutions\n{}".format(resolutions))

        if index in [10, 9, 8, 7, 6, 5, 4, 3, 0]:
            requests.get(url + "/control?var=framesize&val={}".format(index))
        else:
            print("Wrong index")
    except:
        print("SET_RESOLUTION: something went wrong")

def set_quality(url: str, value: int=1, verbose: bool=False):
    try:
        if value >= 10 and value <=63:
            requests.get(url + "/control?var=quality&val={}".format(value))
    except:
        print("SET_QUALITY: something went wrong")

def set_awb(url: str, awb: int=1):
    try:
        awb = not awb
        requests.get(url + "/control?var=awb&val={}".format(1 if awb else 0))
    except:
        print("SET_QUALITY: something went wrong")
    return awb

if __name__ == '__main__':
    set_resolution(URL, index=6)  # Use VGA (640x480) for better performance
    
    # Create a single window with fixed size
    cv2.namedWindow("ESP32 Human Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ESP32 Human Detection", 800, 600)  # Set window size
    
    print("Starting ESP32 Human Detection...")
    print("Press 'r' to change resolution, 'q' to change quality, 'a' to toggle AWB, ESC to quit")
    
    # Performance optimization variables
    frame_count = 0
    detection_skip_frames = 3  # Process every 3rd frame for YOLO
    last_detections = []
    
    import time
    fps_start_time = time.time()

    while True:
        if cap.isOpened():
            ret, frame = cap.read()

            if ret and frame is not None and frame.size > 0:
                frame_count += 1
                
                # Resize frame for faster processing with proper validation
                h, w = frame.shape[:2]
                if h > 240 or w > 320:
                    # Calculate aspect ratio preserving dimensions
                    aspect_ratio = w / h
                    if aspect_ratio > 320/240:  # Width is the limiting factor
                        new_w, new_h = 320, int(320 / aspect_ratio)
                    else:  # Height is the limiting factor
                        new_w, new_h = int(240 * aspect_ratio), 240
                    
                    # Ensure dimensions are even and valid
                    new_w = max(32, (new_w // 2) * 2)
                    new_h = max(32, (new_h // 2) * 2)
                    
                    frame_resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
                else:
                    frame_resized = frame.copy()
                
                # Run YOLO detection only on every Nth frame
                if frame_count % detection_skip_frames == 0:
                    results = human_detector(
                        frame_resized, 
                        conf=0.25,  # Lower confidence threshold for better detection
                        classes=[0],  # class 0 is 'person'
                        verbose=False,
                        imgsz=320,  # Smaller image size for faster processing
                        device='cpu'  # Ensure CPU processing for consistency
                    )
                    
                    # Store detections for reuse
                    last_detections = []
                    if results[0].boxes is not None:
                        boxes = results[0].boxes
                        for box in boxes:
                            x1, y1, x2, y2 = box.xyxy[0].tolist()
                            confidence = float(box.conf[0])
                            
                            # Scale coordinates back to original frame size if needed
                            if h > 240 or w > 320:
                                scale_x = w / new_w
                                scale_y = h / new_h
                                x1, x2 = x1 * scale_x, x2 * scale_x
                                y1, y2 = y1 * scale_y, y2 * scale_y
                            
                            last_detections.append({
                                'bbox': [x1, y1, x2, y2],
                                'confidence': confidence
                            })
                
                # Draw bounding boxes (using cached detections for non-processing frames)
                for detection in last_detections:
                    x1, y1, x2, y2 = detection['bbox']
                    confidence = detection['confidence']
                    
                    # Draw rectangle around detected person
                    frame = cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
                    
                    # Add confidence label
                    label = f"Person: {confidence:.2f}"
                    cv2.putText(frame, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Add FPS counter
                current_time = time.time()
                elapsed = current_time - fps_start_time
                if elapsed >= 1.0:
                    fps = frame_count / elapsed
                    frame_count = 0
                    fps_start_time = current_time
                else:
                    fps = frame_count / max(elapsed, 0.1)
                
                cv2.putText(frame, f"FPS: {fps:.1f} | Humans: {len(last_detections)}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # Resize frame for display to fit window (keeping original resolution for processing)
                display_frame = cv2.resize(frame, (800, 600), interpolation=cv2.INTER_LINEAR)
                
                # Only display frame if it's valid
                cv2.imshow("ESP32 Human Detection", display_frame)
            else:
                print("Warning: No frame received or empty frame")

            key = cv2.waitKey(1)

            if key == ord('r'):
                idx = int(input("Select resolution index: "))
                set_resolution(URL, index=idx, verbose=True)

            elif key == ord('q'):
                val = int(input("Set quality (10 - 63): "))
                set_quality(URL, value=val)

            elif key == ord('a'):
                AWB = set_awb(URL, AWB)

            elif key == 27:
                break

    cv2.destroyAllWindows()
    cap.release()
