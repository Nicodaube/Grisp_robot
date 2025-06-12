import csv
import threading
import datetime
import time
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


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
        
    def print_plots(self):
        self.create_dist_plots()
    
    def create_dist_plots(self, expected_dist1=None, expected_dist2 = None):
        try:
            dist_data = pd.read_csv("./data/sonar_dist_sensor_1_" + self.timestamp + ".csv", header=None, names=["idx", "dist"])
            x1 = dist_data['idx']          
            y11 = dist_data['dist'].astype(float).round(3)        

            dist_data2 = pd.read_csv("./data/sonar_dist_sensor_2_" + self.timestamp + ".csv", header=None, names=["idx", "dist"])
            x2 = dist_data2['idx']
            y12 = dist_data2['dist'].astype(float).round(3)        

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(20, 10), constrained_layout=True)

            ax1.plot(x1, y11, label='Measured distance', linewidth=2, marker='o')
            if expected_dist1 != None :
                y21 = [78 for i in x1]
                ax1.plot(x1, y21, label='True distance', linewidth=2, marker='s')

            ax1.set_title('Variation of sonar measure in sensor_1 over time')
            ax1.set_xlabel('Measure idx')
            ax1.set_ylabel('Distance')
            ax1.set_ylim([0,200])
            ax1.legend()
            ax1.grid(True)

            ax2.plot(x2, y12, label='Measured distance', linewidth=2, marker='o')
            if expected_dist2 != None:
                y22 = [80 for i in x2]
                ax2.plot(x2, y22, label='True distance', linewidth=2, marker='s')
                
            ax2.set_title('Variation of sonar measure in sensor_2 over time')
            ax2.set_xlabel('Measure idx')
            ax2.set_ylabel('Distance')
            ax2.set_ylim([0,200])
            ax2.legend()
            ax2.grid(True)

            plt.savefig('./plots/dist_kalman_' + self.timestamp + '.png')
        except:
            print("[CSV_SAVER] Files not initialized")