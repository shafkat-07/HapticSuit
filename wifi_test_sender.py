import socket
import time
import sys

# Configuration
# Replace this with the IP address printed in the Arduino Serial Monitor
UDP_IP = "10.65.43.146"  
UDP_PORT = 8888

print(f"Target IP: {UDP_IP}")
print(f"Target Port: {UDP_PORT}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(2.0) # 2 second timeout for receiving reply

while True:
    try:
        # Message format: "roll_1,roll_2,throttle_1,throttle_2"
        # Adjusted for 1 IMU/Motor setup: "roll_1, unused, throttle_1, unused"
        # Sending 0 roll target and 50 throttle for the first motor
        message = "0,0,50,0" 
        
        print(f"Sending ping: {message}")
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        
        try:
            data, addr = sock.recvfrom(1024)
            print(f"Received reply: {data.decode().strip()}")
        except socket.timeout:
            print("No reply received (timeout)")
            
        time.sleep(0.5)
        
        # Check if user wants to quit
        # (This is a simplified check to avoid blocking input in this loop for continuous streaming)
        # In a real terminal, you'd just Ctrl+C to stop
            
    except KeyboardInterrupt:
        print("\nExiting...")
        break
    except Exception as e:
        print(f"Error: {e}")
        break
