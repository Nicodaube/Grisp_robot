import csv
import threading
import datetime
import time
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


class CSV_saver:
    def __init__(self):
        plt.switch_backend('Agg')
        date = datetime.datetime.now()
        self.sonar_pos_instantiated = False
        self.sonar_dist_instantiated = False
        self.kalman_pos_instantiated = False
        self.timestamp = str(date.year) + "_" + str(date.month) + "_" + str(date.day) + "_" + str(date.hour) + "_" + str(date.minute)

    def save_robot_pos_sonar(self, x, y, angle, room):
        threading.Thread(target=self.csv_update_pos_sonar, args=(x,y,angle,room), daemon=True).start()

    def csv_update_pos_sonar(self, x, y, angle, room):    
        tmstp = datetime.datetime.now().timestamp()
        if not self.sonar_pos_instantiated:
            self.sonar_pos_timestamp = tmstp
            self.sonar_pos_instantiated = True

        with open("./data/sonar_pos_" + self.timestamp + ".csv", "a") as csv_file:
            fieldnames = ["timestamp", "x", "y", "angle", "room"]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            row = {
                "timestamp": tmstp - self.sonar_pos_timestamp,
                "x": x,
                "y": y,
                "angle": angle,
                "room": room
                }
            
            csv_writer.writerow(row)

    def save_robot_pos_kalman(self, x, y, angle, room):
        threading.Thread(target=self.csv_update_pos_kalman, args=(x,y,angle,room), daemon=True).start()

    def csv_update_pos_kalman(self, x, y, angle, room):
        tmstp = datetime.datetime.now().timestamp()
        if not self.kalman_pos_instantiated:
            self.kalman_pos_timestamp = tmstp
            self.kalman_pos_instantiated = True

        with open("./data/kalman_pos_" + self.timestamp + ".csv", "a") as csv_file:
            fieldnames = ["timestamp", "x", "y", "angle", "room"]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            row = {
                "timestamp": tmstp - self.kalman_pos_timestamp,
                "x": x,
                "y": y,
                "angle": angle,
                "room": room
                }
            
            csv_writer.writerow(row)
    
    def save_distance_sonar(self, name, distance):
        threading.Thread(target=self.csv_update_distance_sonar, args=(name, distance), daemon=True).start()
        threading.Thread(target=self.csv_update_freq_sonar, args=(name,), daemon=True).start()

    def csv_update_distance_sonar(self, name, distance):
        tmstp = datetime.datetime.now().timestamp()
        if not self.sonar_dist_instantiated:
            self.sonar_dist_timestamp = tmstp
            self.sonar_dist_instantiated = True

        with open("./data/sonar_dist_" + name + "_" + self.timestamp + ".csv", "a") as csv_file:
            fieldnames = ["timestamp", "dist"]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            row = {
                "timestamp": tmstp - self.sonar_dist_timestamp,
                "dist": distance
                }
            
            csv_writer.writerow(row)
        
    def print_plots(self):

        try : 
            self.create_dist_plots()
            self.create_pos_kalman_plots()
            self.create_pos_sonar_plots()
        except:
            print("[CSV_SAVER] Files not initialized")
    
    def create_dist_plots(self, expected_dist1=None, expected_dist2 = None):
        dist_data = pd.read_csv("./data/sonar_dist_sensor_1_" + self.timestamp + ".csv", header=None, names=["timestamp", "dist"])
        x1 = dist_data['timestamp']          
        y11 = dist_data['dist'].astype(float).round(3)        

        dist_data2 = pd.read_csv("./data/sonar_dist_sensor_2_" + self.timestamp + ".csv", header=None, names=["timestamp", "dist"])
        x2 = dist_data2['timestamp']
        y12 = dist_data2['dist'].astype(float).round(3)        

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(20, 10), constrained_layout=True)

        ax1.plot(x1, y11, label='Measured distance', linewidth=2, marker='.')
        if expected_dist1 != None :
            y21 = [expected_dist1 for i in x1]
            ax1.plot(x1, y21, label='True distance', linewidth=2, marker='.')

        ax1.set_title('Variation of sonar measure in sensor_1 over time')
        ax1.set_xlabel('Time')
        ax1.set_ylabel('Distance')
        ax1.set_ylim([0,200])
        ax1.legend()
        ax1.grid(True)

        ax2.plot(x2, y12, label='Measured distance', linewidth=2, marker='.')
        if expected_dist2 != None:
            y22 = [expected_dist2 for i in x2]
            ax2.plot(x2, y22, label='True distance', linewidth=2, marker='.')
            
        ax2.set_title('Variation of sonar measure in sensor_2 over time')
        ax2.set_xlabel('Time')
        ax2.set_ylabel('Distance')
        ax2.set_ylim([0,200])
        ax2.legend()
        ax2.grid(True)

        plt.savefig('./plots/dist_kalman_' + self.timestamp + '.png')

    def create_pos_kalman_plots(self):
        pos_data = pd.read_csv("./data/kalman_pos_" + self.timestamp + ".csv", header=None, names=["timestamp", "x", "y", "angle", "room"])
        x = pos_data['timestamp']          
        x_pos = pos_data['x'].astype(float)
            
        y_pos = pos_data['y'].astype(float)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(20, 10), constrained_layout=True)

        ax1.plot(x, x_pos, label='Position on x_axis', linewidth=2, marker='.')
        ax1.set_title('Variation of the position of the robot over time')
        ax1.set_ylabel('Position x axis')
        ax1.set_ylim([0,1])
        ax1.legend()
        ax1.grid(True)

        ax2.plot(x, y_pos, label='Position on y_axis', linewidth=2, marker='.')
        ax2.set_xlabel('Time')
        ax2.set_ylabel('Position Y axis')
        ax2.set_ylim([0,1])
        ax2.legend()
        ax2.grid(True)

        plt.savefig('./plots/pos_kalman_' + self.timestamp + '.png')

    def create_pos_sonar_plots(self):
        pos_data = pd.read_csv("./data/sonar_pos_" + self.timestamp + ".csv", header=None, names=["idx", "x", "y", "angle", "room"])
        x = pos_data['idx']          
        x_pos = pos_data['x'].astype(float)
            
        y_pos = pos_data['y'].astype(float)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(20, 10), constrained_layout=True)

        ax1.plot(x, x_pos, label='Position on x_axis', linewidth=2, marker='.')
        ax1.set_title('Variation of the position of the robot over time')
        ax1.set_ylabel('Position x axis')
        ax1.set_ylim([0,1])
        ax1.legend()
        ax1.grid(True)

        ax2.plot(x, y_pos, label='Position on y_axis', linewidth=2, marker='.')
        ax2.set_xlabel('Measure idx')
        ax2.set_ylabel('Position Y axis')
        ax2.set_ylim([0,1])
        ax2.legend()
        ax2.grid(True)

        plt.savefig('./plots/pos_sonar_' + self.timestamp + '.png')