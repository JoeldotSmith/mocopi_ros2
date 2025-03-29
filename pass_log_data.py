import socket
import time
import base64

REPLAY_FILE = "mocopi_data_log.bin"
UDP_IP = "127.0.0.1"
UDP_PORT = 12351
CHUNK_SIZE = 2048
SEND_INTERVAL = 0.03  # 10 ms per frame

def main():
    with open(REPLAY_FILE, "r") as f:  # Open the file as text
        data_log = f.readlines()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        for line in data_log:
            # Decode the base64-encoded data back to original binary
            decoded_data = base64.b64decode(line.strip())  # Remove newline and decode
            sock.sendto(decoded_data, (UDP_IP, UDP_PORT))  # Send the decoded data
            time.sleep(SEND_INTERVAL)

if __name__ == "__main__":
    main()

