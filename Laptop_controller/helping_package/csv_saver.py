import csv
import threading

class CSV_saver:
    def __init__(self):
        self.sonar_pos_idx = 0
        with open("./data/sonar_pos.csv", "w") as csv:
            pass

    def save_pos_sonar(self, x, y, angle, room):
        threading.Thread(target=self.csv_update_pos_sonar, args=(x,y,angle,room), daemon=True).start()

    def csv_update_pos_sonar(self, x, y, angle, room):
        with open("./data/sonar_pos.csv", "a") as csv_file:
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

            
