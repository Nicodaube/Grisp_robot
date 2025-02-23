import pygame
import sys
import numpy as np
import serial
import time


#Pygame init
pygame.init()
ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
width, height = 800, 600
screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
pygame.display.set_caption("Rotation de la flèche")

# Colors
white = (255, 255, 255)

#load figures
arrow_img = pygame.image.load('./img/arrow.png')
arrow_img = pygame.transform.scale(arrow_img, (arrow_img.get_width() // 2, arrow_img.get_height() // 2))
arrow_rect = arrow_img.get_rect(center=(width // 2, height // 2))
circle_img = pygame.image.load('./img/point.png')
circle_img = pygame.transform.scale(circle_img, (circle_img.get_width() // 2, circle_img.get_height() // 2))
stop_img = pygame.image.load('./img/Stop_sign.png')
stop_img = pygame.transform.scale(stop_img, (stop_img.get_width() // 5, stop_img.get_height() // 5))

font = pygame.font.Font(None, 36)



#state variables
running = True
message = 0
run = True
stand = False
kalman = True
release_space = True
release_enter = True
release_t = True
release_tab = True


test_time = 0
def check_event():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            global running 
            running = False

def check_keys_movement(keys):
    global x
    global running
    global run
    if keys[pygame.K_z] or keys[pygame.K_UP]:
        x += 1
    elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
        x += -1
    elif keys[pygame.K_q] or keys[pygame.K_LEFT]:
        x += 1j
    elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
        x += -1j
    elif keys[pygame.K_ESCAPE]:
        running = False
    elif keys[pygame.K_SPACE]:
        if release_space:
            release_space = False
            if message < 10000000:
                run = True
            else:
                run = False
    else:
        release_space = True

def check_keys_kalman(keys, release_tab):
    global kalman
    if keys[pygame.K_k]:
        kalman = True
    elif keys[pygame.K_c]:
        kalman = False
    elif keys[pygame.K_TAB]:
        if release_tab:
            kalman = not kalman
            return False
    else:
        return True

def check_test(keys, release_t):
    if keys[pygame.K_t] and release_t:
            return True, False
    else:
        return False, True

def build_string():
    global string
    string += "DOWN \n" if not stand else "Up \n"
    string += "Kalman filter\n" if kalman else "Complementary filter\n"
    string += "Running\n" if run else "Stopped\n"
    string += "Message: " + str(message) + "\n"

    for i, line in enumerate(string.split("\n")):
        text = font.render(line, True, (0, 128, 0))
        screen.blit(text, (10, 10 + i * 30))

def update_screen_size():   
    width, height = screen.get_size()
    return arrow_img.get_rect(center=(width // 2, height // 2))

# main loop
while running:
    check_event()
    arrow_rect = update_screen_size()
    
    #Get the keys pressed
    keys = pygame.key.get_pressed()
    x = 0
    string = ""

    check_keys_movement(keys)
    release_tab = check_keys_kalman(keys, release_tab)

    test, release_t = check_test(keys, release_t)

    if keys[pygame.K_RETURN]:
        if release_enter:
            release_enter = False
            stand = not stand
    else:
        release_enter = True

    if test:
        test_time = time.time()

    #if (1<= time.time()-test_time < 10) : stand=True
    #if (10<= time.time()-test_time < 25) : stand=False

    # Clear the screen
    screen.fill(white)

    # Draw the arrow
    if message < 10000000:
        screen.blit(stop_img, stop_img.get_rect(center=arrow_rect.center))
    elif abs(x) == 0:
        screen.blit(circle_img, circle_img.get_rect(center=arrow_rect.center))
    else:
        angle = np.angle(x, deg=True)
        rotated_arrow = pygame.transform.rotate(arrow_img, angle)
        rotated_rect = rotated_arrow.get_rect(center=arrow_rect.center)
        screen.blit(rotated_arrow, rotated_rect.topleft)

    build_string()
    pygame.display.flip()

    # Limit the frame rate
    pygame.time.Clock().tick(200)

    data = run << 7 | kalman << 6 | test << 5 | stand << 4 | (x.real == 1) << 3 | (x.real == -1) << 2 | (
                x.imag == 1) << 1 | (x.imag == -1)
    ser.write(bytes([data]))

    Content = ser.readline()
    Content = Content.decode().replace("\r\n", "")
    message = int(Content)


# Quit
pygame.quit()
sys.exit()
