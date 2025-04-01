import socket
import threading

class Server:

    HOST = '0.0.0.0'
    PORT = 5000
    buffer = []
    sensors_ip = {}
    sensor_data = {}

    def __init__(self):
        self.rcvServer = threading.Thread(target=self.rcv_server, daemon=True)
        self.rcvServer.start()

    def rcv_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            server_socket.bind((self.HOST, self.PORT))
            print(f"[SERVER] Listening for UDP packets on {self.HOST}:{self.PORT}")

            while True:
                data, addr = server_socket.recvfrom(1024)
                data = data.decode()

                if data[:5] == "Hello":
                    id = int(data[11:])
                    self.sensors_ip[id] = addr
                    self.sensor_data[addr[0]] = 0
                    print("[SERVER] Received hello from " + str(id) + " on (" + str(addr[0]) + ", " + str(addr[1]) + ")")
                elif data[:8] == "Distance":
                    self.sensor_data[addr[0]] = float(data[9:])
                else : 
                    print("DATA " + data)

    def send(self, message):
        threading.Thread(target=self.snd_server, args=(message,), daemon=True).start()

    def snd_server(self, message):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            for id, addr in self.sensors_ip.items():
                ip, port = addr[0], addr[1]
                srv_socket.sendto(message.encode(), (ip, port))
                print("[SERVER] Sent to " + str(id) + " on (" + str(ip) + ", " + str(port) + ") : " + str(message))

    def get_sensors(self):
        return self.sensors_ip.keys()



if __name__ == '__main__' :
    serv = Server()
