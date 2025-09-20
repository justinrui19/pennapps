#!/usr/bin/env python3
"""
ESP32-CAM Network Discovery and Connection Tool
Finds and connects to ESP32-CAM on the network
"""

import subprocess
import requests
import socket
import time
import threading
from concurrent.futures import ThreadPoolExecutor
import cv2
import numpy as np

def scan_network_for_esp32():
    """Scan local network for ESP32-CAM devices"""
    print("Scanning local network for ESP32-CAM devices...")
    
    # Get local network range
    try:
        # Get default gateway
        result = subprocess.run(['route', 'get', 'default'], 
                              capture_output=True, text=True)
        gateway_line = [line for line in result.stdout.split('\n') 
                       if 'gateway:' in line][0]
        gateway = gateway_line.split(':')[1].strip()
        
        # Assume /24 network (most common)
        network_base = '.'.join(gateway.split('.')[:-1])
        print(f"Scanning network: {network_base}.0/24")
        
    except:
        # Fallback to common ranges
        network_base = "192.168.1"
        print(f"Using fallback network: {network_base}.0/24")
    
    def check_esp32_device(ip):
        """Check if IP has ESP32-CAM"""
        try:
            response = requests.get(f"http://{ip}", timeout=2)
            if response.status_code == 200:
                content = response.text.lower()
                if any(keyword in content for keyword in ['esp32', 'camera', 'cam']):
                    return ip
        except:
            pass
        return None
    
    # Scan IPs in parallel
    possible_ips = [f"{network_base}.{i}" for i in range(1, 255)]
    
    with ThreadPoolExecutor(max_workers=50) as executor:
        futures = [executor.submit(check_esp32_device, ip) for ip in possible_ips]
        
        found_devices = []
        for future in futures:
            result = future.result()
            if result:
                found_devices.append(result)
                print(f"✓ Found ESP32-CAM at: {result}")
    
    return found_devices

def test_esp32_ports(ip):
    """Test common ESP32-CAM ports"""
    common_ports = [80, 81, 8080, 8081, 1234]
    accessible_ports = []
    
    print(f"Testing ports on {ip}...")
    
    for port in common_ports:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((ip, port))
            sock.close()
            
            if result == 0:
                accessible_ports.append(port)
                print(f"  ✓ Port {port}: Open")
                
                # Test HTTP on this port
                try:
                    response = requests.get(f"http://{ip}:{port}", timeout=3)
                    content_type = response.headers.get('content-type', '')
                    print(f"    HTTP Status: {response.status_code}")
                    print(f"    Content-Type: {content_type}")
                    
                    if 'multipart' in content_type or 'image' in content_type:
                        print(f"    → Possible stream endpoint!")
                        
                except Exception as e:
                    print(f"    HTTP test failed: {e}")
            else:
                print(f"  ✗ Port {port}: Closed")
                
        except Exception as e:
            print(f"  ✗ Port {port}: Error - {e}")
    
    return accessible_ports

def test_stream_endpoints(ip, port):
    """Test various stream endpoints"""
    endpoints = ['', '/stream', '/capture', '/cam', '/video', '/mjpeg']
    
    print(f"Testing stream endpoints on {ip}:{port}...")
    
    working_endpoints = []
    
    for endpoint in endpoints:
        url = f"http://{ip}:{port}{endpoint}"
        try:
            response = requests.get(url, timeout=5, stream=True)
            content_type = response.headers.get('content-type', '')
            
            print(f"  {endpoint or '/'}: Status {response.status_code}, Type: {content_type}")
            
            if response.status_code == 200:
                if 'multipart' in content_type or 'image' in content_type:
                    working_endpoints.append((endpoint, content_type))
                    print(f"    ✓ Stream endpoint found!")
            
            response.close()
            
        except requests.exceptions.Timeout:
            print(f"  {endpoint or '/'}: Timeout")
        except Exception as e:
            print(f"  {endpoint or '/'}: Error - {e}")
    
    return working_endpoints

def quick_stream_test(url):
    """Quick test to verify stream is working"""
    print(f"Testing stream: {url}")
    
    try:
        response = requests.get(url, stream=True, timeout=10)
        
        if response.status_code != 200:
            print(f"✗ Bad status: {response.status_code}")
            return False
        
        # Try to read some data
        chunk = response.raw.read(4096)
        if len(chunk) > 0:
            print(f"✓ Stream is sending data ({len(chunk)} bytes)")
            
            # Look for JPEG markers
            if b'\xff\xd8' in chunk:
                print("✓ JPEG data detected")
                return True
            else:
                print("? Data format unclear")
                return True
        else:
            print("✗ No data received")
            return False
            
    except Exception as e:
        print(f"✗ Stream test failed: {e}")
        return False

if __name__ == "__main__":
    print("ESP32-CAM Network Discovery Tool")
    print("=" * 40)
    
    # First, try the known IP
    known_ip = "192.168.43.36"
    print(f"Testing known IP: {known_ip}")
    
    try:
        response = requests.get(f"http://{known_ip}", timeout=5)
        print(f"✓ Known IP is accessible: {response.status_code}")
        
        # Test ports
        ports = test_esp32_ports(known_ip)
        
        # Test stream endpoints on each port
        for port in ports:
            endpoints = test_stream_endpoints(known_ip, port)
            
            # Test each working endpoint
            for endpoint, content_type in endpoints:
                url = f"http://{known_ip}:{port}{endpoint}"
                if quick_stream_test(url):
                    print(f"\n🎉 Working stream found: {url}")
                    print(f"Content-Type: {content_type}")
                    break
    
    except Exception as e:
        print(f"✗ Known IP failed: {e}")
        print("\nScanning network for ESP32-CAM devices...")
        
        # Scan for devices
        devices = scan_network_for_esp32()
        
        if devices:
            print(f"\nFound {len(devices)} ESP32-CAM device(s)")
            for device in devices:
                print(f"\nTesting device: {device}")
                ports = test_esp32_ports(device)
                
                for port in ports:
                    endpoints = test_stream_endpoints(device, port)
                    
                    for endpoint, content_type in endpoints:
                        url = f"http://{device}:{port}{endpoint}"
                        if quick_stream_test(url):
                            print(f"\n🎉 Working stream found: {url}")
                            print(f"Content-Type: {content_type}")
                            break
        else:
            print("No ESP32-CAM devices found on network")