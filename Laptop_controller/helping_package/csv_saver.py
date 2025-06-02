import csv
import threading
import datetime

class CSV_saver:
    def __init__(self):
        date = datetime.datetime.now()
        self.sonar_pos_idx = 0
        self.timestamp = str(date.year) + "_" + str(date.month) + "_" + str(date.day) + "_" + str(date.hour) + "_" + str(date.minute)

    def save_pos_sonar(self, x, y, angle, room):
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

            
