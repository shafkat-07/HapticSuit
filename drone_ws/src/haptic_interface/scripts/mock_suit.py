#!/usr/bin/env python3
import socket
import sys
import time
from datetime import datetime

UDP_IP = "127.0.0.1"
UDP_PORT = 8888

def start_mock_suit():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        sock.bind((UDP_IP, UDP_PORT))
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Mock Suit Receiver Started")
        print(f"Listening on {UDP_IP}:{UDP_PORT}...")
        print("---------------------------------------------------------")
        print("Waiting for haptic packets...")
        
        while True:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            message = data.decode().strip()
            
            # Timestamp
            ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            
            print(f"[{ts}] Received: {message}")
            
            # Optional: Try to parse if it matches the expected CSV format
            try:
                parts = message.split(',')
                if len(parts) == 8:
                    # q1,q2,q3,q4,w1,w2,w3,w4
                    print(f"    Angles (deg):   {float(parts[0]):.1f}, {float(parts[1]):.1f}, {float(parts[2]):.1f}, {float(parts[3]):.1f}")
                    print(f"    Throttles (%):  {float(parts[4]):.1f}, {float(parts[5]):.1f}, {float(parts[6]):.1f}, {float(parts[7]):.1f}")
            except ValueError:
                pass
                
    except OSError as e:
        print(f"Error binding to port {UDP_PORT}: {e}")
        print("Make sure no other instance (or the real bridge) is using this port.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopping Mock Suit.")
        sock.close()
        sys.exit(0)

if __name__ == "__main__":
    start_mock_suit()

