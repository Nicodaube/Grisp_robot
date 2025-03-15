import socket

HOST = '0.0.0.0'
PORT = 5000

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
    server_socket.bind((HOST, PORT))
    print(f"Listening for UDP packets on {HOST}:{PORT}")

    while True:
        data, addr = server_socket.recvfrom(1024)  # Receive UDP packet
        print(f"Received from {addr}: {data.decode()}")
