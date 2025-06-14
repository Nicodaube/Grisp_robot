import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

pos_data = pd.read_csv("./sonar_pos_2025_6_12_12_26.csv", header=None, names=["idx", "x", "y", "angle", "room"])
x = pos_data['idx']          
x_pos = pos_data['x'].astype(float)
       
y_pos = pos_data['y'].astype(float)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(20, 10), constrained_layout=True)

ax1.plot(x, x_pos, label='Position on x_axis', linewidth=2)
ax1.set_title('Variation of the position of the robot over time')
ax1.set_ylabel('Position')
ax1.set_ylim([0,1])
ax1.legend()
ax1.grid(True)

ax2.plot(x, y_pos, label='Position on y_axis', linewidth=2)
ax2.set_xlabel('Measure idx')
ax2.set_ylabel('Position')
ax2.set_ylim([0,1])
ax2.legend()
ax2.grid(True)

plt.show()
#plt.savefig('../plots/dist_multiplex.png')
