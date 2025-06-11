import socket
import threading
from components.Sensor import Sensor
from components.Robot import Robot
from helping_package.csv_saver import CSV_saver
import time

class Server:

    HOST = '172.20.10.8'
    PORT = 5000
    buffer = []
    sensors = {}
    room_edges = []
    started = False

    def __init__(self):
        self.robot = Robot()
        self.rcvServer = threading.Thread(target=self.rcv_server, daemon=True)
        self.rcvServer.start()
        self.pinger = threading.Thread(target=self.ping_server, daemon=True)
        self.pinger.start()

        self.csv_saver = CSV_saver()

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

    def rcv_server(self): # Processes the messages received from the different devices
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            server_socket.bind((self.HOST, self.PORT))
            print(f"[SERVER] Listening for UDP packets on {self.HOST}:{self.PORT}")

            while True:
                data, addr = server_socket.recvfrom(1024)
                try :
                    data = data.decode()       
                    data_split = data.split(",")    

                    if data_split[0] == "Hello": # Received from a device when it has discovered the sever
                        self.handle_hello(data_split, addr)                        
                    elif data_split[0] == "Robot_pos": # Received from devices at each iteration of the kalman measure (only saves robot update)
                        self.update_robot_pos(data, addr)                        
                    elif data_split[0] == "Ack": # Received from devices after each configuration message
                        self.handle_ack(data)   
                    elif data_split[0] == "Distance":
                        self.csv_saver.save_distance_sonar(data_split[2], data_split[1])
                    else : # Default case
                        print("[SERVER] received strange data : " + data)
                except : 
                    pass

#==========================================================================================================================================
#===================================================== RECEIVE SERVER FUNCTIONS ===========================================================
#==========================================================================================================================================

    def handle_hello(self, data, addr):
        # Processes the hello message received, creates new sensor in the case of a sensor, updates robot otherwise
        # @param data : the decoded data received (String)
        # @param addr : the Ip address and Port associated with the received message (List)

        id = data[1]
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
        # Updates the robot position based on the message received
        # @param data : the decoded data received (String)
        # @param addr : the Ip address and Port associated with the received message (List)

        data_split = data.strip().split(",")
        if addr[0] == self.robot.ip:
            self.csv_saver.save_robot_pos_sonar(float(data_split[1]), float(data_split[2]), int(data_split[3]), int(data_split[4]))
            self.robot.update_pos(float(data_split[1]), float(data_split[2]), int(data_split[3]), int(data_split[4]))

    def handle_ack(self, data):
        # Processes the ack message, updates the ack list
        # @param data : the decoded data received (String)

        data_split = data.split(",")
        config_message = data_split[1]
        id = data_split[2]
        origin = data_split[3]
        
        if origin != "robot":                                                
            if config_message == "Pos":
                self.ack_pos.get(id)[int(origin)-1] = True   
            elif config_message == "Room_info":
                self.ack_rooms.get(int(id))[int(origin)-1] = True
            else :
                self.ack_devices.get(id)[int(origin)-1] = True
        else :
            if config_message == "Pos":
                self.ack_pos.get(id)[len(self.sensors.keys())] = True   
            elif config_message == "Room_info":
                self.ack_rooms.get(int(id))[len(self.sensors.keys())] = True
            else :
                self.ack_devices.get(id)[len(self.sensors.keys())] = True   
        print("[SERVER] Received Ack " + config_message + " for " + id + " from " + origin)   

#==========================================================================================================================================
#============================================================= SEND SERVERS ===============================================================
#==========================================================================================================================================

    def send(self, message, type, id=None):
        # Creates a sending server thread of the specified type and passes arguments
        # @param message: the message to be sent (String)
        # @param type: the way the message has to be sent, can be "brd" for broadcast and "uni" for unicast (String)
        # @param id: the identifier of the device to update (String, Integer, None)

        if type == "brd":
            threading.Thread(target=self.brd_server, args=(message,), daemon=True).start()
        elif type == "uni":
            threading.Thread(target=self.uni_server, args=(message, id), daemon=True).start()

        if message == "Exit":
            self.csv_saver.print_plots()

    def brd_server(self, message):
        # Sends a broadcast message
        # @param message: the message to be sent (String)

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            srv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            broadcast_ip = '172.20.10.15'
            port = 9000            
            srv_socket.sendto(message.encode(), (broadcast_ip, port))
            if message[:4] != "ping":
                print(f"[SERVER] Broadcasted to ({broadcast_ip}, {port}): {message}")

    def uni_server(self, message, id):
        # Sends a unicast message to the specified device
        # @param message: the message to be sent (String)
        # @param id: the identifier of the device to update (String, Integer)

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

    def send_config(self): # Creates a worker_send_config thread
        threading.Thread(target=self.worker_send_config, daemon=True).start()

    def worker_send_config(self): # Sends the whole config to all the devices to setup the system
        self.started = True
        self.ack_devices = {}
        self.ack_pos = {}
        self.ack_rooms = {}

        for sensor in self.sensors.values() :
            sensor_config_ok = self.send_sensor_infos(sensor)

        if sensor_config_ok:
            room_config_ok = self.send_rooms_infos()

        if room_config_ok:
            self.send_robot_info()
        
            time.sleep(1)
            message = "Start " + self.HOST
            self.send(message, "brd")
        else :
            self.send("Exit", "brd")

    def send_sensor_infos(self, sensor):
        # Sends all the informations about a sensor to all the devices
        # @param sensor: the actual sensor from which we draw the info (Sensor)
        # @return a ack boolean if the informations of this sensor where successfully delivered to everyone, false otherwise

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

                ack = self.check_ack("sensor_" + str(sensor.id), "sensor")
                LIMIT += 1
        
        return ack

    def send_robot_info(self): # Sends all the informations about the robot to all the devices
        if self.robot.ip != "0":
            ack = False
            LIMIT = 0
            while (not ack) and (LIMIT < 10):
                message = "Add_Device : robot , " + self.robot.ip + " , " + str(self.robot.port)
                self.send(message, "brd")
                time.sleep(0.5)
                message = "Init_pos : " + str(self.robot.real_pos[0]) + " , " + str(self.robot.real_pos[1]) + " , " + str(self.robot.angle) + " , " + str(self.robot.room)
                self.send(message, "brd")
                
                ack = self.check_ack("robot", "sensor")
                LIMIT +=1

    def send_rooms_infos(self):
        
        for room_idx in range(len(self.room_edges)):
            self.ack_rooms[room_idx] = [False for i in range(len(self.sensors.keys())+1)]
            ack = False

            LIMIT = 0
            while (not ack) and (LIMIT < 10):
                message = "Room_info," + str(room_idx) 
                message += "," + str(self.room_edges[room_idx][0][0]) 
                message += "," + str(self.room_edges[room_idx][0][1])
                message += "," + str(self.room_edges[room_idx][1][0])
                message += "," + str(self.room_edges[room_idx][1][1])
                self.send(message, "brd")
                time.sleep(0.5)

                ack = self.check_ack(room_idx, "room")
                LIMIT += 1
            
            if not ack:
                return False
        return True

    def check_ack(self, id, type): 
        # Checks that all devices have acknowledged all the informations about the current device 
        # @return True if all devices acked, False otherwise

        if type == "sensor":
            for i in range(len(self.sensors.keys())+1):
                if not self.ack_devices.get(id)[i]:
                    return False
                if not self.ack_pos.get(id)[i]:
                    return False
            return True
        elif type == "room":
            for i in range(len(self.sensors.keys())+1):
                if not self.ack_rooms.get(id)[i]:
                    return False
            return True

#==========================================================================================================================================
#============================================================= API FUNCTIONS ==============================================================
#==========================================================================================================================================

    def get_sensors(self):
        # @return all the known sensors ids 

        return self.sensors.keys()

    def update_sens(self, id, side, room, x, y):
        # Updates the position of a sensor in a room
        # @param id: the id of the sensor (Integer)
        # @param side: the side or corner in which the sensor has been placed (String)
        # @param room: the room in which the sensor has been placed (Integer)
        # @param x: the x-axis position of the sensor in the grid (float)
        # @param y: the y-axis position of the sensor in the grid (float)

        sens = self.sensors.get(id)
        sens.set_angle(side)
        sens.update_pos(room, x, y)
        
    def update_sens_height(self, id, height):
        # update the height of the sensor
        # @param id: the id of the sensor to modify (Integer)
        # @param height: the height of the sensor (Float)

        self.sensors.get(id).update_height(height)

    def update_robot(self, real_pos, angle, room):
        # Updates the robot position
        # @param real_pos: the position of the robot on the grid (Tuple)
        # @param angle: the angle of the robot (0 <= Integer <= 360)
        # @param room: the room in which the robot is placed (Integer)
        
        self.robot.update_pos(real_pos[0], real_pos[1], angle, room.room_num)

    def add_edges(self, TLpos, BRpos):

        self.room_edges.append((TLpos, BRpos))