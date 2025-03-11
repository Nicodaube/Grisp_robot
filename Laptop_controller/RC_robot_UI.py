import pygame
import sys
import numpy as np
import serial
import time

class User_interface:
    WIDTH, HEIGHT = 800, 600
    running = True
    message = 0
    run = True
    stand = False
    kalman = True
    release_space = True
    release_enter = True
    release_t = True
    release_tab = True
    x = 0
    string = ""
    image_dict = {}

    def __init__(self):

        pygame.init()
        self.ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
        
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption("Rotation de la flèche")

        self.load_figures()

    def load_figures(self):
        self.arrow_img = pygame.image.load('Laptop_controller/img/arrow.png')
        self.arrow_img = pygame.transform.scale(self.arrow_img, (self.arrow_img.get_width() // 4, self.arrow_img.get_height() // 4))
        self.circle_img = pygame.image.load('Laptop_controller/img/point.png')
        self.circle_img = pygame.transform.scale(self.circle_img, (self.circle_img.get_width() // 2, self.circle_img.get_height() // 2))
        self.stop_img = pygame.image.load('Laptop_controller/img/Stop_sign.png')
        self.stop_img = pygame.transform.scale(self.stop_img, (self.stop_img.get_width() // 10, self.stop_img.get_height() // 10))

        self.image_dict["arrow"] = self.arrow_img
        self.image_dict["circle"] = self.circle_img
        self.image_dict["stop"] = self.stop_img

######################################################### TRIGGER CHECK #################################################

    def check_event(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

######################################################### KEYBOARD FUNCTIONS #################################################

    def check_keys_movement(self, keys):
        if keys[pygame.K_z] or keys[pygame.K_UP]:
            self.x += 1
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
            self.x += -1
        elif keys[pygame.K_q] or keys[pygame.K_LEFT]:
            self.x += 1j
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            self.x += -1j
        elif keys[pygame.K_ESCAPE]:
            self.running = False
        elif keys[pygame.K_SPACE]:
            if self.release_space:
                self.release_space = False
                if self.message < 10000000:
                    self.run = True
                else:
                    self.run = False
        else:
            self.release_space = True

    def check_keys_kalman(self, keys):
        if keys[pygame.K_k]:
            self.kalman = True
        elif keys[pygame.K_c]:
            self.kalman = False
        elif keys[pygame.K_TAB]:
            if self.release_tab:
                self.kalman = not self.kalman
                self.release_tab = False
        else:
            self.release_tab = True

    def check_test(self, keys):
        if keys[pygame.K_t] and self.release_t:
                self.test, self.release_t = True, False
        else:
            self.test, self.release_t =  False, True

    def check_standing(self, keys):
        if keys[pygame.K_RETURN]:
            if self.release_enter:
                self.stand = not self.stand
                self.release_enter = False
                
        else:
            self.release_enter = True

######################################################### DRAWING FUNCTIONS #################################################

    def update_screen_size(self):   
        self.WIDTH, self.HEIGHT = self.screen.get_size()
        return self.image_dict.get("arrow").get_rect(center=(self.WIDTH // 2, 100))

    def draw_move_ctrl(self):
        control_rect = self.image_dict.get("arrow").get_rect(center=(self.WIDTH // 2, 100))
        if self.message < 10000000:
            self.screen.blit(self.image_dict.get("stop"), self.image_dict.get("stop").get_rect(center = control_rect.center))
        elif abs(self.x) == 0:
            self.screen.blit(self.image_dict.get("circle"), self.image_dict.get("circle").get_rect(center = control_rect.center))
        else:
            angle = np.angle(self.x, deg=True)
            rotated_arrow = pygame.transform.rotate(self.image_dict.get("arrow"), angle)
            rotated_rect = rotated_arrow.get_rect(center = control_rect.center)
            self.screen.blit(rotated_arrow, rotated_rect.topleft)

    def draw_string(self):
        font = pygame.font.Font(None, 36)
        self.string += "DOWN \n" if not self.stand else "UP \n"
        self.string += "Kalman filter\n" if self.kalman else "Complementary filter\n"
        self.string += "Running\n" if self.run else "Stopped\n"
        self.string += "Message: " + str(self.message) + "\n"

        for i, line in enumerate(self.string.split("\n")):
            text = font.render(line, True, (0, 128, 0))
            self.screen.blit(text, (10, 10 + i * 30))

######################################################### SERIAL COMM FUNCTIONS #################################################

    def serial_comm(self):
        data = self.run << 7 | self.kalman << 6 | self.test << 5 | self.stand << 4 | (self.x.real == 1) << 3 | (self.x.real == -1) << 2 | (
                    self.x.imag == 1) << 1 | (self.x.imag == -1)
        self.ser.write(bytes([data]))

        Content = self.ser.readline()
        Content = Content.decode().replace("\r\n", "")
        self.message = int(Content)

######################################################### MAIN LOOP ############################################################
    def main_loop(self):
        while self.running:
            self.check_event()
            self.update_screen_size()
            
            keys = pygame.key.get_pressed()
            self.x = 0
            self.string = ""
            self.check_keys_movement(keys)
            self.check_keys_kalman(keys)
            self.check_test(keys)
            self.check_standing(keys)

            self.screen.fill((255, 255, 255))

            self.draw_move_ctrl()
            self.draw_string()

            pygame.display.flip()

            # Limit the frame rate
            pygame.time.Clock().tick(200)

            self.serial_comm()


        # Quit
        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    ui = User_interface()
    ui.main_loop()