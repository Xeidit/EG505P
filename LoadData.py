import numpy as np
import matplotlib.pyplot as plt

# Load the saved data
Data = np.load("sensor_data.npy")

# Restore into your plot
fig, ax = plt.subplots()
map = ax.scatter([], [])
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
map.set_offsets(Data)
plt.show()
