import socket
import threading

class Server:

    HOST = '0.0.0.0'
    PORT = 5000
    buffer = []
    sensors_ip = {}

    def __init__(self):
        #Create or clear the file for sensors ip
        file = open("./sensors_data/sensors_ip.txt", "w")
        file.close() 

        self.rcvServer = threading.Thread(target=self.rcv_server(), daemon=True)
        self.sendServer = threading.Thread(target=self.snd_server(), daemon=True)

    def rcv_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            server_socket.bind((self.HOST, self.PORT))
            print(f"Listening for UDP packets on {self.HOST}:{self.PORT}")

            while True:
                data, addr = server_socket.recvfrom(1024)
                data = data.decode()

                if data[:5] == "Hello":
                    self.sensors_ip[data[7:]] = addr

    def snd_server(self):
        pass

    def get_Sensors(self):
        return self.sensors_ip




