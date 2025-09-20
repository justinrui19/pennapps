#!/usr/bin/env python3
import time
import math
import random
import requests
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', type=str, default='http://localhost:8000', help='Dashboard server base URL')
    parser.add_argument('--rate', type=float, default=0.5, help='Seconds between updates')
    args = parser.parse_args()

    t0 = time.time()
    hazard_kinds = ['fire', 'smoke', 'debris']

    print(f"Sending simulated events to {args.server} ... Ctrl+C to stop")

    while True:
        now = time.time() - t0

        # Pose moves in a circle
        pose = {
            't': now,
            'position': [math.cos(now/5.0)*5.0, math.sin(now/5.0)*5.0, 0.0],
            'orientation': [0.0, 0.0, math.sin(now/10.0), math.cos(now/10.0)]
        }
        try:
            requests.post(f"{args.server.rstrip('/')}/api/pose", json=pose, timeout=1)
        except Exception:
            pass

        # Detections: 0-3 random people with stable ids p_0..p_2
        num = random.randint(0, 3)
        detections = []
        for i in range(num):
            x, y, w, h = random.randint(50, 550), random.randint(50, 350), 60, 120
            detections.append({
                'id': f'p_{i}',
                'bbox': [x, y, x + w, y + h],
                'confidence': round(random.uniform(0.5, 0.95), 2),
                'source': 'sim'
            })
        try:
            requests.post(f"{args.server.rstrip('/')}/api/detections", params={'replace': 'true'}, json=detections, timeout=1)
        except Exception:
            pass

        # Occasionally spawn/update a random hazard with stable id
        if random.random() < 0.2:
            hid = random.choice(['h_fire', 'h_smoke', 'h_debris'])
            kind = hid.split('_')[1]
            hazard = {
                'id': hid,
                'kind': kind,
                'position': [random.uniform(-10, 10), random.uniform(-10, 10), 0.0],
                'confidence': round(random.uniform(0.6, 0.99), 2)
            }
            try:
                requests.post(f"{args.server.rstrip('/')}/api/hazards", json=[hazard], timeout=1)
            except Exception:
                pass

        time.sleep(args.rate)


if __name__ == '__main__':
    main()
