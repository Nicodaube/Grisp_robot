class Sensor :

    def __init__(self, IP, Port, ID):
        self.ip = IP
        self.port = Port
        self.id = ID
        self.room = -1
        self.x = -1
        self.y = -1
        self.distance = -1
        self.height = 0

    def update_pos(self, room, x, y):
        self.room = room
        self.x = x
        self.y = y 
        print("[SENSOR_" + str(self.id) + "] is at (" + str(self.x) + ", " + str(self.y) + ") in room number " + str(self.room))

    def update_data(self, distance):
        self.distance = distance

    def update_height(self, height):
        print("[SENSOR_" + str(self.id) + "] is at " + str(height) +"m from the ground")
        self.height = height