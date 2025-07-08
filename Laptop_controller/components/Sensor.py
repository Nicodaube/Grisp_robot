class Sensor :

    def __init__(self, IP, Port, ID):
        self.ip = IP
        self.port = Port
        self.id = ID
        self.room = -1
        self.x = -1
        self.y = -1
        self.height = 0
        self.angle = 0
        self.distance = -1
        
    def set_angle(self, side):
        match side:
            case "L":            
                self.angle = 0
            case "TL":
                self.angle = 45
            case "T":            
                self.angle = 90
            case "TR":
                self.angle = 135
            case "R":            
                self.angle = 180
            case "BR":
                self.angle = 225
            case "B":            
                self.angle = 270
            case "BL":
                self.angle = 315
    
    def update_pos(self, room, x_corner, y_corner):
        self.room = room

        match self.angle:
            case 0:  
                x = x_corner
                y = y_corner                
            case 45:
                x = round(x_corner + 0.12, 2)
                y = round(y_corner - 0.16, 2) 
                                
            case 90:   
                x = x_corner
                y = y_corner                
            case 135:
                x = round(x_corner - 0.16, 2)
                y = round(y_corner - 0.12, 2)
            case 180:     
                x = x_corner
                y = y_corner                
            case 225:
                x = round(x_corner - 0.16, 2)
                y = round(y_corner + 0.12, 2)
            case 270:
                x = x_corner
                y = y_corner       
            case 315:
                x = round(x_corner + 0.16, 2)
                y = round(y_corner + 0.12, 2)
        self.x = x
        self.y = y 
        print("[SENSOR_" + str(self.id) + "] is at (" + str(self.x) + ", " + str(self.y) + ") in room number " + str(self.room) + " with an angle of " + str(self.angle))

    def update_data(self, distance):
        self.distance = distance

    def update_height(self, height):
        print("[SENSOR_" + str(self.id) + "] is at " + str(height) +"m from the ground")
        self.height = height