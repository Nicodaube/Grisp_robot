from Side import Side

class Room:

    def __init__(self, width, height, x, y, room_num):
        self.width = width
        self.height = height
        self.pos = (x,y)
        self.sides = {
            "L":None,
            "R":None,
            "T":None,
            "B":None
                    }
        self.room_num = room_num
        
    def modify_side(self, side, img, img_type):
        x, y = self.compute_pos(side)
        self.sides[side] = Side(x, y, img, img_type)

    def compute_pos(self, side):
        if side == "L":
            side_x, side_y = self.pos[0] - self.width//2, self.pos[1]  
        elif side == "R":
            side_x, side_y = self.pos[0] + self.width//2, self.pos[1]  
        elif side == "T":
            side_x, side_y = self.pos[0], self.pos[1] + self.height//2
        elif side == "B":
            side_x, side_y = self.pos[0], self.pos[1] - self.height//2

        return side_x, side_y

        
    def add_room(self, side, room):
        self.sides[side] = room

    

