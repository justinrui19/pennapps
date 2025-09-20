#!/usr/bin/env python3
import subprocess
import sys
import os
import signal
import time


def run():
    if len(sys.argv) < 2:
        print("Usage: orchestrator.py <server_base_url> [stream_url]")
        print("Example: orchestrator.py http://localhost:8000 http://192.168.43.36:81/stream")
        sys.exit(1)
    server = sys.argv[1]
    stream = sys.argv[2] if len(sys.argv) >= 3 else "http://192.168.43.36:81/stream"

    env = os.environ.copy()

    # YOLO detections -> dashboard
    yolo_cmd = [sys.executable, os.path.join(os.path.dirname(__file__), 'esp32_yolo_detection.py'), '--server', server]

    # SLAM -> dashboard (pose)
    slam_cmd = [sys.executable, os.path.join(os.path.dirname(__file__), 'run_esp32_slam.py'), server]

    procs = []
    try:
        print("Starting YOLO detections...")
        procs.append(subprocess.Popen(yolo_cmd, env=env))
        time.sleep(1)
        print("Starting SLAM...")
        procs.append(subprocess.Popen(slam_cmd, env=env))
        # Wait
        while True:
            time.sleep(1)
            # If any exits, break
            for p in list(procs):
                if p.poll() is not None:
                    raise RuntimeError("One of the processes exited")
    except KeyboardInterrupt:
        print("\nStopping orchestrator...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass
        for p in procs:
            try:
                p.wait(timeout=5)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass


if __name__ == '__main__':
    run()
