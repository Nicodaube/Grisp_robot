import socket

HOST = '0.0.0.0'
PORT = 5000

#Create or clear the file for sensors ip
file = open("./sensors_data/sensors_ip.txt", "w")
file.close() 

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
    server_socket.bind((HOST, PORT))
    print(f"Listening for UDP packets on {HOST}:{PORT}")

    while True:
        data, addr = server_socket.recvfrom(1024)
        data = data.decode()

        if data[:5] == "Hello":
            with open("./sensors_data/sensors_ip.txt", "a") as ip_file :
                ip_file.write(addr)