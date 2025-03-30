from Room import Room

def parse(filename, current_height, current_resize):
    rooms = []
    with open("./saves/" + filename, "r") as file:
        lines = file.readlines()
        resize = int(lines[0][16:])
        height = int(lines[1][9:])
        for line in lines[2:]:
            rooms.append(createRoom(line, height, current_height, resize, current_resize))
    return rooms


def createRoom(line, height, current_height, resize, current_resize):
    splitted = line.split(", ")
    new_width, new_height = compute_current_size(int(splitted[0]), int(splitted[1]), height, current_height, resize, current_resize)
    room = Room(new_width,
                new_height,
                int(splitted[2][1:]),
                int(splitted[3][:-1]),
                int(splitted[4]))
    
    for side in ["L", "R", "T", "B"]:
        room.modify_side(side, "./img/plus.png", "plus")

    return room
                
                     
def compute_current_size(width, height, past_height, current_height,  past_resize, current_resize):
   new_width, new_height = int(width / (past_height//past_resize)), int(height / (past_height//past_resize))
   new_width, new_height = int(new_width * (current_height//current_resize)), int(new_height * (current_height//current_resize))
   return new_width, new_height
