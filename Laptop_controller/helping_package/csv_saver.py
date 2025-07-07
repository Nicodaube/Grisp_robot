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
        self.clock_updater_instantiated = False
        self.sonar_pos_instantiated = False
        self.sonar_dist_instantiated = False
        self.kalman_pos_instantiated = False
        self.timestamp = str(date.year) + "_" + str(date.month) + "_" + str(date.day) + "_" + str(date.hour) + "_" + str(date.minute)

    def save_clock(self, num, clock):
        threading.Thread(target=self.csv_update_clock, args=(num, clock), daemon=True).start()

    def csv_update_clock(self, num, clock):
        tmstp = datetime.datetime.now().timestamp()

        with open("./data/clock_tick_" + self.timestamp + ".csv", "a") as csv_file:
            fieldnames = ["timestamp", "num", "clock"]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            row = {
                "timestamp": tmstp,
                "num": num,
                "clock": clock
                }
            
            csv_writer.writerow(row)

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

    def csv_update_distance_sonar(self, name, distance):
        tmstp = datetime.datetime.now().timestamp()

        with open("./data/sonar_dist_" + name + "_" + self.timestamp + ".csv", "a") as csv_file:
            fieldnames = ["timestamp", "dist"]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            row = {
                "timestamp": tmstp,
                "dist": distance
                }
            
            csv_writer.writerow(row)
        
    def print_plots(self):
        try : 
            self.create_dist_plots()
            self.create_pos_kalman_plots()
        except:
            print("[CSV_SAVER] Error: something went wrong printing the plots")
    

    def create_dist_plots(self):

        df_clock = pd.read_csv(f"./data/clock_tick_{self.timestamp}.csv", header=None, names=["timestamp", "num", "clock"])
        df1 = pd.read_csv(f"./data/sonar_dist_sensor_1_{self.timestamp}.csv",header=None, names=["timestamp", "dist"])
        df2 = pd.read_csv(f"./data/sonar_dist_sensor_2_{self.timestamp}.csv",header=None, names=["timestamp", "dist"])
        t0 = df_clock["timestamp"].iloc[0]

        df_clock["timestamp"] -= t0
        df1["timestamp"] -= t0
        df2["timestamp"] -= t0

        xt = df_clock["timestamp"]
        x1 = df1["timestamp"]
        y1 = df1["dist"].astype(float).round(4)
        x2 = df2["timestamp"]
        y2 = df2["dist"].astype(float).round(4)

        fig, ax = plt.subplots(figsize=(20, 10))
        ax.plot(x1, y1, label="Measured distance sensor 1", linewidth=1, marker='.')
        ax.plot(x2, y2, label="Measured distance sensor 2", linewidth=1, marker='.')

        ax.set_ylim([0, 100])
        ax.set_yticks(np.arange(0, 100, step=10))
        ymin, ymax = ax.get_ylim()
        ax.vlines(xt, ymin, ymax, linewidth=0.25, label="Clock tick", colors="red")

        ax.set_title("Sonar Distance Measurements with Clock Ticks")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Ground distance (cm)")

        ax.legend()
        ax.grid(True)
        
        plt.savefig(f"./plots/dist_kalman_{self.timestamp}.png")

    def create_pos_kalman_plots(self):
        pos_data = pd.read_csv(f"./data/kalman_pos_{self.timestamp}.csv", header=None, names=["timestamp", "x", "y", "angle", "room"])
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

        plt.savefig(f'./plots/pos_kalman_{self.timestamp}.png')