import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# ─── Replace these placeholder lists with your actual data ───────────────────
dist_data = pd.read_csv("./dist_kalman_s1_V2.csv", header=None, names=["idx", "dist"])
x1 = dist_data['idx']          # Common x‐axis for the two curves
y11 = dist_data['dist'].astype(float).round(3)                   # First curve’s y‐values
y21 = [78 for i in x1]                    # Second curve’s y‐values

dist_data2 = pd.read_csv("./dist_kalman_s2_V2.csv", header=None, names=["idx", "dist"])
x2 = dist_data2['idx']          # Common x‐axis for the two curves
y12 = dist_data2['dist'].astype(float).round(3)                   # First curve’s y‐values
y22 = [80 for i in x2]                    # Second curve’s y‐values

# ─────────────────────────────────────────────────────────────────────────────

# 1. Create a figure with two subplots (stacked vertically)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), constrained_layout=True)

# 2. First subplot: plot two curves
ax1.plot(x1, y11, label='Measured distance', linewidth=2, marker='o')
ax1.plot(x1, y21, label='True distance', linewidth=2, marker='s')
ax1.set_title('Variation of sonar measure in sensor_1 over time')
ax1.set_xlabel('Measure idx')
ax1.set_ylabel('Distance')
ax1.legend()
ax1.grid(True)

# 3. Second subplot: plot frequency vs. time
ax2.plot(x2, y12, label='Measured distance', linewidth=2, marker='o')
ax2.plot(x2, y22, label='True distance', linewidth=2, marker='s')
ax2.set_title('Variation of sonar measure in sensor_2 over time')
ax2.set_xlabel('Measure idx')
ax2.set_ylabel('Distance')
ax2.legend()
ax2.grid(True)

# 4. Show the combined figure
plt.show()
