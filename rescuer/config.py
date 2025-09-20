import os


def get_default_stream_url() -> str:
    return os.getenv("ESP32_STREAM_URL", "http://192.168.43.36:81/stream")


def get_default_camera_base() -> str:
    url = get_default_stream_url()
    # Expect ...://host:81/stream
    try:
        base = url.split('://', 1)[1]
        host = base.split('/', 1)[0]
        host_only = host.split(':', 1)[0]
        return f"http://{host_only}"
    except Exception:
        return "http://192.168.43.36"


def get_default_server_base() -> str:
    return os.getenv("DASHBOARD_BASE", "http://localhost:8000")


