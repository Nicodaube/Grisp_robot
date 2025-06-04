import csv
import threading
import datetime
import time

class CSV_saver:
    def __init__(self):
        date = datetime.datetime.now()
        self.sonar_pos_idx = 0
        self.sonar_dist_idx = 0
        self.sonar_freq_idx = 0
        self.timestamp = str(date.year) + "_" + str(date.month) + "_" + str(date.day) + "_" + str(date.hour) + "_" + str(date.minute)

    def save_robot_pos_sonar(self, x, y, angle, room):
        threading.Thread(target=self.csv_update_pos_sonar, args=(x,y,angle,room), daemon=True).start()

    def csv_update_pos_sonar(self, x, y, angle, room):
        with open("./data/sonar_pos_" + self.timestamp + ".csv", "a") as csv_file:
            fieldnames = ["idx", "x", "y", "angle", "room"]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            row = {
                "idx": self.sonar_pos_idx,
                "x": x,
                "y": y,
                "angle": angle,
                "room": room
                }
            
            csv_writer.writerow(row)
            self.sonar_pos_idx += 1
    
    def save_distance_sonar(self, name, distance):
        threading.Thread(target=self.csv_update_distance_sonar, args=(name, distance), daemon=True).start()
        threading.Thread(target=self.csv_update_freq_sonar, args=(name,), daemon=True).start()

    def csv_update_distance_sonar(self, name, distance):
        with open("./data/sonar_dist_" + name + "_" + self.timestamp + ".csv", "a") as csv_file:
            fieldnames = ["idx", "dist"]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            row = {
                "idx": self.sonar_dist_idx,
                "dist": distance
                }
            
            csv_writer.writerow(row)
            self.sonar_dist_idx += 1

    def csv_update_freq_sonar(self, name):
        with open("./data/sonar_freq_" + name + "_" + self.timestamp + ".csv", "a") as csv_file:
            fieldnames = ["idx", "timestamp"]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            row = {
                "idx": self.sonar_freq_idx,
                "timestamp": time.time()
                }
            
            csv_writer.writerow(row)
            self.sonar_freq_idx += 1

            
