#!/usr/bin/env python3
import argparse
import time
from typing import List, Dict

import cv2
import numpy as np
import requests
from ultralytics import YOLO


def parse_args():
    p = argparse.ArgumentParser(description="Test dual YOLO models (human + fire) on one video")
    p.add_argument('--video', type=str, required=True, help='Path to input video file')
    p.add_argument('--human-model', type=str, default='yolov8n.pt', help='YOLO model for human detection')
    p.add_argument('--fire-model', type=str, default='yolov8n_fire.pt', help='YOLO model for fire/smoke')
    p.add_argument('--imgsz', type=int, default=640, help='Inference image size')
    p.add_argument('--conf-human', type=float, default=0.25, help='Confidence threshold for humans')
    p.add_argument('--conf-fire', type=float, default=0.6, help='Confidence threshold for fire')
    p.add_argument('--blocklist', type=str, default='kite', help='Comma-separated class names to ignore (case-insensitive)')
    p.add_argument('--skip', type=int, default=1, help='Process every Nth frame')
    p.add_argument('--save', type=str, default='', help='Optional output video path (e.g., out.mp4)')
    p.add_argument('--server', type=str, default='', help='Optional backend base URL to publish detections/hazards')
    p.add_argument('--post-interval', type=float, default=0.5, help='Seconds between posts if --server is set')
    return p.parse_args()


def draw_box(frame, xyxy, color, label):
    x1, y1, x2, y2 = map(int, xyxy)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
    cv2.putText(frame, label, (x1 + 3, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter = inter_w * inter_h
    area_a = max(0.0, (ax2 - ax1)) * max(0.0, (ay2 - ay1))
    area_b = max(0.0, (bx2 - bx1)) * max(0.0, (by2 - by1))
    denom = area_a + area_b - inter
    return inter / denom if denom > 0 else 0.0


def _nms(dets: list, iou_th: float = 0.5):
    if not dets:
        return dets
    dets_sorted = sorted(dets, key=lambda d: d.get('confidence', 0.0), reverse=True)
    kept = []
    for d in dets_sorted:
        suppress = False
        for k in kept:
            if _iou(d['bbox'], k['bbox']) >= iou_th:
                suppress = True
                break
        if not suppress:
            kept.append(d)
    return kept


def main():
    args = parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {args.video}")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(args.save, fourcc, fps, (width, height))

    # Load models
    human_model = YOLO(args.human_model)
    fire_model = YOLO(args.fire_model)

    frame_idx = 0
    last_post = 0.0
    cv2.namedWindow('Dual YOLO Test', cv2.WINDOW_NORMAL)

    # prepare blocklist set
    blocklist = set([s.strip().lower() for s in (args.blocklist or '').split(',') if s.strip()])

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_idx += 1

        process = (frame_idx % args.skip) == 0
        humans: List[Dict] = []
        fires: List[Dict] = []

        if process:
            # Humans (COCO class 0)
            hr = human_model(
                frame, imgsz=args.imgsz, conf=args.conf_human,
                classes=[0], verbose=False, device='cpu'
            )
            hboxes = getattr(hr[0], 'boxes', None)
            if hboxes is not None:
                for box in hboxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = float(box.conf[0])
                    humans.append({'bbox': [x1, y1, x2, y2], 'confidence': conf})

            # Fire (custom classes) - accept all classes
            fr = fire_model(frame, imgsz=args.imgsz, conf=args.conf_fire, verbose=False, device='cpu')
            fboxes = getattr(fr[0], 'boxes', None)
            names = getattr(fr[0], 'names', None)
            if names is None:
                names = getattr(fire_model, 'names', None)
            if fboxes is not None:
                for box in fboxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0]) if box.cls is not None else -1
                    # Resolve class name from model metadata; fallback to class id
                    if isinstance(names, dict):
                        cname = names.get(cls, str(cls))
                    elif isinstance(names, (list, tuple)) and 0 <= cls < len(names):
                        cname = names[cls]
                    else:
                        cname = str(cls)
                    # Hotfix mapping: treat 'laptop' class as 'fire'
                    if isinstance(cname, str) and cname.lower() == 'laptop':
                        cname = 'fire'
                    # Skip unwanted classes
                    if isinstance(cname, str) and cname.lower() in blocklist:
                        continue
                    fires.append({'bbox': [x1, y1, x2, y2], 'confidence': conf, 'label': cname})

        # Post-process to remove duplicate person boxes (NMS)
        humans = _nms(humans, iou_th=0.55)

        # Draw overlays
        for d in humans:
            draw_box(frame, d['bbox'], (0, 200, 0), f"Person {d['confidence']:.2f}")
        for d in fires:
            lbl = (d.get('label') or 'fire').capitalize()
            draw_box(frame, d['bbox'], (20, 80, 255), f"{lbl} {d['confidence']:.2f}")

        # HUD text
        cv2.putText(frame, f"Frame {frame_idx} | Humans: {len(humans)} | Fire: {len(fires)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Optional publishing
        now = time.time()
        if args.server and (now - last_post) >= args.post_interval:
            try:
                # post humans as detections
                det_payload = []
                for i, d in enumerate(humans):
                    det_payload.append({
                        'id': f'p_{i}',
                        'bbox': d['bbox'],
                        'confidence': d['confidence'],
                        'source': 'video'
                    })
                if det_payload:
                    requests.post(f"{args.server.rstrip('/')}/api/detections", params={'replace': 'true'}, json=det_payload, timeout=0.5)

                # post fires as hazards (no depth → z=0)
                haz_payload = []
                for i, d in enumerate(fires):
                    haz_payload.append({
                        'id': f'fire_{i}',
                        'kind': (d.get('label') or 'fire'),
                        'position': [0.0, 0.0, 0.0],
                        'confidence': d['confidence']
                    })
                if haz_payload:
                    requests.post(f"{args.server.rstrip('/')}/api/hazards", json=haz_payload, timeout=0.5)
                last_post = now
            except Exception:
                pass

        # Output
        if writer is not None:
            writer.write(frame)
        cv2.imshow('Dual YOLO Test', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


