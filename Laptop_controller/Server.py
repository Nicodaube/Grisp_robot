import socket
import threading
from components.Sensor import Sensor
from components.Robot import Robot
import time

class Server:

    HOST = '172.20.10.8'
    PORT = 5000
    buffer = []
    sensors = {}
    started = False

    def __init__(self):
        self.robot = Robot()
        self.rcvServer = threading.Thread(target=self.rcv_server, daemon=True)
        self.rcvServer.start()
        self.pinger = threading.Thread(target=self.ping_server, daemon=True)
        self.pinger.start()

#==========================================================================================================================================
#===================================================== PINGER SERVER ======================================================================
#==========================================================================================================================================

    def ping_server(self): # Broadcasts a ping message every 3 seconds until the system is started
        while not self.started : 
            time.sleep(3)
            message = "ping : server , " + self.HOST + " , " + str(self.PORT)
            self.send(message, "brd")
            

#==========================================================================================================================================
#===================================================== RECEIVE SERVER =====================================================================
#==========================================================================================================================================

    def rcv_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            server_socket.bind((self.HOST, self.PORT))
            print(f"[SERVER] Listening for UDP packets on {self.HOST}:{self.PORT}")

            while True:
                data, addr = server_socket.recvfrom(1024)
                try :
                    data = data.decode()               

                    if data[:5] == "Hello": # Received from a device when it has discovered the sever
                        self.handle_hello(data, addr)                        
                    elif data[:9] == "Robot_pos": # Received from devices at each iteration of the kalman measure (only saves robot update)
                        self.update_robot_pos(data, addr)                        
                    elif data[:3] == "Ack": # Received from devices after each configuration message
                        self.handle_ack(data)   
                    else :
                        print("[SERVER] received strange data : " + data)
                except : 
                    pass

#==========================================================================================================================================
#===================================================== RECEIVE SERVER FUNCTIONS ===========================================================
#==========================================================================================================================================

    def handle_hello(self, data, addr):
        id = data[11:]
        if id == "robot":
            print("[SERVER] Received hello from Robot on (" + addr[0] + ", " + str(addr[1]) + ")")
            self.robot.update_adress(addr[0], addr[1])
            self.send("Ack, server", "uni", "robot")
        else : 
            id = int(id)
            self.sensors[id] = Sensor(addr[0], addr[1], id)
            print("[SERVER] Received hello from sensor_" + str(id) + " on (" + str(addr[0]) + ", " + str(addr[1]) + ")")
            self.send("Ack , server", "uni", id)

    def update_robot_pos(self, data, addr):
        data_split = data.strip().split(",")
        if addr[0] == self.robot.ip:
            self.robot.update_pos(float(data_split[1]), float(data_split[2]), int(data_split[3]), int(data_split[4]))

    def handle_ack(self, data):
        data_split = data.split(",")
        config_message = data_split[1]
        id = data_split[2]
        origin = data_split[3]
        
        if origin != "robot":                                                
            if config_message == "Pos":
                self.ack_pos.get(id)[int(origin)-1] = True   
            else :
                self.ack_devices.get(id)[int(origin)-1] = True
        else :
            if config_message == "Pos":
                self.ack_pos.get(id)[len(self.sensors.key())] = True   
            else :
                self.ack_devices.get(id)[len(self.sensors.key())] = True   
        print("[SERVER] Received Ack " + config_message + " for " + id + " from " + origin)   

#==========================================================================================================================================
#============================================================= SEND SERVERS ===============================================================
#==========================================================================================================================================

    def send(self, message, type, id=None):
        if type == "brd":
            threading.Thread(target=self.brd_server, args=(message,), daemon=True).start()
        elif type == "uni":
            threading.Thread(target=self.uni_server, args=(message, id), daemon=True).start()

    def brd_server(self, message):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            srv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            broadcast_ip = '172.20.10.15'
            port = 9000            
            srv_socket.sendto(message.encode(), (broadcast_ip, port))
            if message[:4] != "ping":
                print(f"[SERVER] Broadcasted to ({broadcast_ip}, {port}): {message}")

    def uni_server(self, message, id):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            if id == "robot":
                ip = self.robot.ip
                port = self.robot.port      
                srv_socket.sendto(message.encode(), (ip, port))   
                print("[SERVER] Sent to Robot on (" + str(ip) + ", " + str(port) + ") : " + str(message))       
            else : 
                sensor = self.sensors.get(id)
                ip = sensor.ip
                port = sensor.port
                srv_socket.sendto(message.encode(), (ip, port))
                print("[SERVER] Sent to " + str(sensor.id) + " on (" + str(ip) + ", " + str(port) + ") : " + str(message))

#==========================================================================================================================================
#============================================================= SEND CONFIG LOGIC ==========================================================
#========================================================================================================================================== 

    def send_config(self):
        threading.Thread(target=self.worker_send_config, daemon=True).start()

    def worker_send_config(self):
        self.started = True
        self.ack_devices = {}
        self.ack_pos = {}

        for sensor in self.sensors.values() :
            sensor_config_ok = self.send_sensor_info(sensor)

        if sensor_config_ok:
            self.send_robot_info()
        
            time.sleep(1)
            message = "Start " + self.HOST
            self.send(message, "brd")
        else :
            self.send("Exit", "brd")

    def send_sensor_info(self, sensor):
        # Init ack status 
        self.ack_devices["sensor_" + str(sensor.id)] = [False for i in range(len(self.sensors.keys()) + 1)]
        self.ack_devices["robot"] = [False for i in range(len(self.sensors.keys()) + 1)]
        self.ack_pos["sensor_" + str(sensor.id)] = [False for i in range(len(self.sensors.keys()) + 1)]
        self.ack_pos["robot"] = [False for i in range(len(self.sensors.keys()) + 1)]

        if sensor.x != -1 :
            ack = False
            LIMIT = 0
            while (not ack) and (LIMIT < 10): 
                message = "Add_Device : sensor_" + str(sensor.id) + " , " + sensor.ip + " , " + str(sensor.port)                
                self.send(message, "brd")
                time.sleep(0.5)
                message = "Pos " + str(sensor.id) + " : " + str(sensor.x) + " , " + str(sensor.y) + " , " + str(sensor.height) + " , " + str(sensor.angle) + " , " + str(sensor.room)
                self.send(message, "brd")
                time.sleep(0.5)

                ack = self.check_ack("sensor_" + str(sensor.id))
                LIMIT += 1
        return ack

    def send_robot_info(self):
        if self.robot.ip != "0":
            ack = False
            LIMIT = 0
            while (not ack) and (LIMIT < 10):
                message = "Add_Device : robot , " + self.robot.ip + " , " + str(self.robot.port)
                self.send(message, "brd")
                time.sleep(0.5)
                message = "Init_pos : " + str(self.robot.real_pos[0]) + " , " + str(self.robot.real_pos[1]) + " , " + str(self.robot.angle) + " , " + str(self.robot.room)
                self.send(message, "brd")
                
                ack = self.check_ack("robot")
                LIMIT +=1

    def check_ack(self, id):
        for i in range(len(self.sensors.keys())):
            if not self.ack_devices.get(id)[i]:
                return False
            if not self.ack_pos.get(id)[i]:
                return False
        return True

#==========================================================================================================================================
#============================================================= API FUNCTIONS ==============================================================
#==========================================================================================================================================

    def get_sensors(self):
        return self.sensors.keys()

    def update_sens(self, id, side, room, x, y):
        sens = self.sensors.get(id)
        sens.set_angle(side)
        sens.update_pos(room, x, y)
        
    def update_sens_height(self, id, height):
        self.sensors.get(id).update_height(height)

    def update_robot(self, real_pos, angle, room):
        self.robot.update_pos(real_pos[0], real_pos[1], angle, room.room_num)