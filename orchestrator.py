#!/usr/bin/env python3
import subprocess
import sys
import os
import signal
import time
from rescuer.config import get_default_stream_url, get_default_server_base


def run():
    server = get_default_server_base()
    stream = get_default_stream_url()
    if len(sys.argv) >= 2:
        server = sys.argv[1]
    if len(sys.argv) >= 3:
        stream = sys.argv[2]

    env = os.environ.copy()

    # YOLO detections -> dashboard
    yolo_cmd = [sys.executable, os.path.join(os.path.dirname(__file__), 'esp32_yolo_detection.py'), '--server', server, '--stream', stream]

    # SLAM -> dashboard (pose)
    slam_cmd = [sys.executable, os.path.join(os.path.dirname(__file__), 'run_esp32_slam.py'), '--server', server, '--stream', stream]

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
