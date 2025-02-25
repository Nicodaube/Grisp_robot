import pygame
import pygame_gui
import sys
import numpy as np
import serial
import time

class User_interface:
    WIDTH, HEIGHT = 1200, 1000
    running = True
    message = 0
    run = True
    stand = False
    kalman = True
    release_space = True
    release_enter = True
    release_t = True
    release_tab = True
    in_popup = False
    active_popup = None
    UI_elements = []
    temp_origin = None
    
    x = 0
    string = ""
    image_dict = {}
    rect_dict = {}

    rooms = []
    

    def __init__(self):

        pygame.init()
        self.ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
        
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption("Rotation de la flèche")

        self.manager = pygame_gui.UIManager((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()
        self.clock.tick(200)

        self.load_figures()

    def load_figures(self):
        arrow_img = pygame.image.load('./img/arrow.png')
        arrow_img = pygame.transform.scale(arrow_img, (arrow_img.get_width() // 4, arrow_img.get_height() // 4))

        circle_img = pygame.image.load('./img/point.png')
        circle_img = pygame.transform.scale(circle_img, (circle_img.get_width() // 2, circle_img.get_height() // 2))

        stop_img = pygame.image.load('./img/Stop_sign.png')
        stop_img = pygame.transform.scale(stop_img, (stop_img.get_width() // 10, stop_img.get_height() // 10))

        plus_img = pygame.image.load('./img/plus.png')
        plus_img = pygame.transform.scale(plus_img, (plus_img.get_width() // 5, plus_img.get_height() // 5))

        minus_img = pygame.image.load('./img/minus.png')
        minus_img = pygame.transform.scale(minus_img, (minus_img.get_width() // 5, minus_img.get_height() // 5))

        self.image_dict["arrow"] = arrow_img
        self.image_dict["circle"] = circle_img
        self.image_dict["stop"] = stop_img
        self.image_dict["plus_L_0"] = plus_img
        self.image_dict["minus"] = minus_img

######################################################### TRIGGER CHECK #################################################

    def check_event(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                if len(self.rooms) == 0 and self.is_click_image("plus_L_0", event) and not self.in_popup:
                    self.in_popup = True
                    self.temp_origin = "plus_L_0"
                    self.create_room_popup()

                for room in range(len(self.rooms)):
                    for side in ["L", "R", "T", "B"]:
                        name = "plus_" + side + "_" + str(room)
                        if self.is_click_image(name, event) and not self.in_popup:
                            self.in_popup = True
                            self.temp_origin = name
                            self.create_room_popup()
                
            if event.type == pygame_gui.UI_BUTTON_PRESSED:
                width = self.UI_elements[0].get_text()
                height = self.UI_elements[1].get_text()
                self.UI_elements = []
                if width != "" and height != "":
                    self.rooms.append((float(width), float(height), self.WIDTH//2, self.HEIGHT//2, self.temp_origin))

                self.temp_origin = None

                self.active_popup.kill()

                self.active_popup = None
                self.in_popup = False

            self.manager.process_events(event)

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
        self.manager.set_window_resolution((self.WIDTH, self.HEIGHT))

    def draw_move_ctrl(self):
        if self.message < 10000000:
            self.draw_image("stop", self.WIDTH //2, 100)
        elif abs(self.x) == 0:
            self.draw_image("circle", self.WIDTH //2, 100)
        else:
            angle = np.angle(self.x, deg=True)
            rotated_arrow = pygame.transform.rotate(self.image_dict.get("arrow"), angle)
            rotated_rect = rotated_arrow.get_rect(center = (self.WIDTH//2, 100))
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

    def draw_add_room(self):
        if len(self.rooms) == 0:
            self.draw_image("plus_L_0", self.WIDTH//2, self.HEIGHT//2)
        else :
            i = 0
            for (width, height, x, y, origin) in self.rooms:
                adapted_width = int(width * (self.HEIGHT//6))
                adapted_height = int(height * (self.HEIGHT//6))

                plus_L_x = x - adapted_width//2
                plus_L_y = y            

                plus_R_x = x + adapted_width//2
                plus_R_y = y
                
                plus_T_x = x 
                plus_T_y = y + adapted_height//2
                
                plus_B_x = x 
                plus_B_y = y - adapted_height//2
                
                self.load_new_images(i)

                self.draw_image("plus_L_"+str(i), plus_L_x, plus_L_y)
                self.draw_image("plus_R_"+str(i), plus_R_x, plus_R_y)
                self.draw_image("plus_T_"+str(i), plus_T_x, plus_T_y)
                self.draw_image("plus_B_"+str(i), plus_B_x, plus_B_y)

    def draw_image(self, name, x, y):
        plus_rect = self.image_dict.get(name).get_rect(center = (x, y))
        self.screen.blit(self.image_dict.get(name), self.image_dict.get(name).get_rect(center=plus_rect.center))
        self.rect_dict[name] = plus_rect

    def draw_room(self, width, height, x, y, origin):
        adapted_width = int(width * (self.HEIGHT//6))
        adapted_height = int(height * (self.HEIGHT//6))

        room_rect = pygame.Rect(0, 0, adapted_width, adapted_height)
        room_rect.center = (x, y)

        pygame.draw.rect(self.screen, (0, 0, 0), room_rect, width=10)

    def create_room_popup(self):
        # Calculate sizes for buttons and popup dimensions
        button_width = self.WIDTH // 2 - self.WIDTH // 20
        button_height = min(self.HEIGHT // 20, 60)
        popup_width = self.WIDTH // 2
        popup_height = self.HEIGHT // 3
        margin_left = (self.WIDTH - button_width)//20
        margin = 20

        # Center the popup on the screen
        popup_rect = pygame.Rect(
            (self.WIDTH - popup_width) // 2,
            (self.HEIGHT - popup_height) // 2,
            popup_width,
            popup_height
        )

        popup_window = pygame_gui.elements.UIWindow(
            rect=popup_rect,
            manager=self.manager,
            window_display_title='Room Creation'
        )

        self.active_popup = popup_window

        current_y = margin

        header_label = pygame_gui.elements.UILabel(
            
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Enter the room size:",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        width_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Width (m):",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements.append(pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            manager=self.manager,
            container=popup_window
        ))
        current_y += button_height + margin

        height_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Height (m):",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements.append(pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            manager=self.manager,
            container=popup_window
        ))
        current_y += button_height + margin

        self.popup_submit_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Submit",
            manager=self.manager,
            container=popup_window
        )

        self.manager.draw_ui(self.screen)
        pygame.display.update()

######################################################### SERIAL COMM FUNCTIONS #################################################

    def serial_comm(self):
        data = self.run << 7 | self.kalman << 6 | self.test << 5 | self.stand << 4 | (self.x.real == 1) << 3 | (self.x.real == -1) << 2 | (
                    self.x.imag == 1) << 1 | (self.x.imag == -1)
        self.ser.write(bytes([data]))

        Content = self.ser.readline()
        Content = Content.decode().replace("\r\n", "")
        try :
            self.message = int(Content)
        except:
            print("ERROR: serial connection")

############################################################ HELPER FUNCTION #####################################################
    def is_click_image(self, name, event):
        return self.rect_dict.get(name) != None and self.rect_dict.get(name).collidepoint(event.pos)
    
    def load_new_images(self,room_num):
        plus_img = pygame.image.load('./img/plus.png')
        plus_img = pygame.transform.scale(plus_img, (plus_img.get_width() // 5, plus_img.get_height() // 5))

        for side in ["L", "R", "T", "B"]:
            name = "plus_" + side + "_" + str(room_num)
            self.image_dict[name] = plus_img

######################################################### MAIN LOOP ############################################################
    def main_loop(self):
        while self.running:
            self.check_event()
            self.update_screen_size()

            keys = pygame.key.get_pressed()
            self.x = 0
            self.string = ""

            if not self.in_popup:            

                self.check_keys_movement(keys)
                self.check_keys_kalman(keys)
                self.check_test(keys)
                self.check_standing(keys)

            self.screen.fill((255, 255, 255))

            for (width, height, x, y, origin) in self.rooms:
                self.draw_room(width, height, x, y, origin)

            self.draw_move_ctrl()
            self.draw_add_room()
            self.draw_string()



            self.manager.update(self.clock.tick(60)/1000)
            self.manager.draw_ui(self.screen)

            pygame.display.flip()

            self.serial_comm()

        # Quit
        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    ui = User_interface()
    ui.main_loop()