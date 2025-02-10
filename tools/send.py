import socket
import time

# ESP32's IP address (replace with actual IP)
ESP32_IP = "192.168.1.103"  # Change this to your ESP32's IP
UDP_PORT = 2222

# Paths to the binary files
BIN_FILES = ["output1.bin", "output2.bin"]

# Transmission parameters

PACKET_SIZE = 256  # Safe UDP packet size (≤ 1472 bytes)


# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_files():
    try:
        while True:
            for bin_file in BIN_FILES:
                try:
                    with open(bin_file, "rb") as f:
                        while True:
                            data = f.read(PACKET_SIZE)
                            if not data:
                                break  # End of file
                            
                            sock.sendto(data, (ESP32_IP, UDP_PORT))  # Send binary data
                            time.sleep(0.5)  # Delay to match baud rate

                    print(f"Finished sending {bin_file}, moving to next file...")

                except FileNotFoundError:
                    print(f"Error: {bin_file} not found.")
                
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    send_files()