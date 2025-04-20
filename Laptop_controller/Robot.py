class Robot:

    def __init__(self, pos, real_pos, angle, room):
        self.pos = pos
        self.real_pos = real_pos
        self.angle = angle
        self.room = room
        self.confirmed = False