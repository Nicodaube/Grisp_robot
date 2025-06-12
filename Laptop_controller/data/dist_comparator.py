import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

dist_data = pd.read_csv("./dist_s1_multiplex.csv", header=None, names=["idx", "dist"])
x1 = dist_data['idx']          
y11 = dist_data['dist'].astype(float).round(3)

dist_data2 = pd.read_csv("./dist_s2_multiplex.csv", header=None, names=["idx", "dist"])
x2 = dist_data2['idx']
y12 = dist_data2['dist'].astype(float).round(3)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(20, 10), constrained_layout=True)

ax1.plot(x1, y11, label='Measured distance', linewidth=2, marker='o')
ax1.set_title('Variation of sonar measure in sensor_1 over time')
ax1.set_xlabel('Measure idx')
ax1.set_ylabel('Distance')
ax1.set_ylim([0,200])
ax1.legend()
ax1.grid(True)

ax2.plot(x2, y12, label='Measured distance', linewidth=2, marker='o')
ax2.set_title('Variation of sonar measure in sensor_2 over time')
ax2.set_xlabel('Measure idx')
ax2.set_ylabel('Distance')
ax2.set_ylim([0,200])
ax2.legend()
ax2.grid(True)

plt.savefig('../plots/dist_multiplex.png')
