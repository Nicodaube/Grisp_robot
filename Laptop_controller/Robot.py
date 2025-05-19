class Robot:
    real_pos = (0,0)
    angle = 0
    room = -1
    ip = "1"
    port = 0

    def __init__(self):
        self.confirmed = False

    def update_adress(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port

    def update_pos(self, x:float, y: float, angle: int, room: int) -> None:
        print("[ROBOT] is at " + str((x, y)) + " with an angle of " + str(angle) + " in room " + str(room))
        self.real_pos = (x, y)
        self.angle = angle
        self.room = room
