import numpy as np
import matplotlib.pyplot as plt

# Load the saved data
pos_data = np.load("pos_data.npy")
out_data = np.load("out_data.npy")

# Restore into plot
fig, ax = plt.subplots()
map = ax.scatter([], [])
loc = ax.scatter([], [])
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
loc.set_offsets(pos_data)
map.set_offsets(out_data)
plt.show()
