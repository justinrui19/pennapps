#!/usr/bin/env python3
"""
Simple ESP32-CAM stream test
"""

import requests
import socket
import time

URL = "http://192.168.43.36"
STREAM_URL = "http://192.168.43.36:81"

def test_port_81():
    """Test if port 81 is open"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex(('192.168.43.36', 81))
        sock.close()
        if result == 0:
            print("✓ Port 81 is open")
            return True
        else:
            print("✗ Port 81 is closed")
            return False
    except Exception as e:
        print(f"✗ Port test failed: {e}")
        return False

def test_http_methods():
    """Try different HTTP methods"""
    methods = ['GET', 'HEAD', 'POST']
    
    for method in methods:
        try:
            print(f"Trying {method} request to port 81...")
            if method == 'GET':
                response = requests.get(STREAM_URL, timeout=3, stream=True)
            elif method == 'HEAD':
                response = requests.head(STREAM_URL, timeout=3)
            elif method == 'POST':
                response = requests.post(STREAM_URL, timeout=3)
            
            print(f"  {method}: Status {response.status_code}")
            if hasattr(response, 'headers'):
                content_type = response.headers.get('content-type', 'unknown')
                print(f"  Content-Type: {content_type}")
            
            response.close()
            return True
            
        except requests.exceptions.Timeout:
            print(f"  {method}: Timeout")
        except Exception as e:
            print(f"  {method}: Error - {e}")
    
    return False

def test_stream_activation():
    """Try to activate the stream first"""
    activation_urls = [
        URL + "/control?var=stream&val=1",
        URL + "/stream",
        URL + "/capture",
        URL + "/start_stream"
    ]
    
    for url in activation_urls:
        try:
            print(f"Trying activation URL: {url}")
            response = requests.get(url, timeout=3)
            print(f"  Status: {response.status_code}")
            if response.status_code == 200:
                print("  ✓ Activation successful, testing stream...")
                time.sleep(1)
                # Now test the stream
                try:
                    stream_response = requests.get(STREAM_URL, timeout=5, stream=True)
                    print(f"  Stream status: {stream_response.status_code}")
                    stream_response.close()
                    return True
                except:
                    print("  Stream still not accessible")
        except Exception as e:
            print(f"  Failed: {e}")
    
    return False

if __name__ == "__main__":
    print("ESP32-CAM Stream Diagnostics")
    print("=" * 30)
    
    # Test basic connectivity
    try:
        response = requests.get(URL, timeout=5)
        print(f"✓ Base URL accessible: {response.status_code}")
    except Exception as e:
        print(f"✗ Base URL failed: {e}")
        exit(1)
    
    # Test port 81
    test_port_81()
    
    # Test different HTTP methods
    print("\nTesting HTTP methods:")
    test_http_methods()
    
    # Try stream activation
    print("\nTrying stream activation:")
    test_stream_activation()