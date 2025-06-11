import pandas as pd
import matplotlib.pyplot as plt
import os

def compute_mean_frequency(csv_path):
    """
    Read a CSV file containing two columns (idx, timestamp) with no header,
    compute the mean frequency as:
        (idx_last – idx_first) / (timestamp_last – timestamp_first)
    Returns the mean frequency (float, in Hz).
    """
    # Read CSV assuming no header, two columns: idx, timestamp
    df = pd.read_csv(csv_path, header=None, names=["idx", "timestamp"])
    
    # Ensure numeric types
    df["idx"] = df["idx"].astype(float)
    df["timestamp"] = df["timestamp"].astype(float)
    
    # Sort by idx in case the file is not in order
    df = df.sort_values(by="idx").reset_index(drop=True)
    
    idx_first = df["idx"].iloc[0]
    idx_last  = df["idx"].iloc[-1]
    t_first   = df["timestamp"].iloc[0]
    t_last    = df["timestamp"].iloc[-1]
    
    elapsed = t_last - t_first
    if elapsed <= 0:
        return float("nan")
    
    # Number of “samples” = difference in idx
    num_samples = idx_last - idx_first
    mean_freq = num_samples / elapsed
    return mean_freq

# ─── Replace these with your actual CSV file paths ─────────────────────────────
file_paths = [
    "freq_s1_V1.csv",
    "freq_s1_V2.csv",
    "freq_s1_V3.csv",
]
# ────────────────────────────────────────────────────────────────────────────────

labels = ["V1:100ms/300", "V2:200ms/300", "V3:200ms/20"]
frequencies = []

for path in file_paths:
    freq = compute_mean_frequency(path)
    frequencies.append(freq)

# Create a bar plot of mean frequencies
plt.figure(figsize=(6, 4))
bars = plt.bar(labels, frequencies, color=["C0", "C1", "C2"])
plt.ylabel("Mean Frequency (Hz)")
plt.title("Mean Computation Frequency for Each CSV")

# Annotate each bar with its frequency value (rounded to 2 decimals)
for bar, freq in zip(bars, frequencies):
    height = bar.get_height()
    if not pd.isna(freq):
        plt.text(
            bar.get_x() + bar.get_width()/2,
            height + 0.01 * max(frequencies),
            f"{freq:.2f}",
            ha="center",
            va="bottom"
        )

plt.ylim(0, max(frequencies)*1.1)
plt.grid(axis="y", linestyle="--", alpha=0.5)
plt.tight_layout()
plt.show()
