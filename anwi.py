import matplotlib.pyplot as plt
import numpy
fig, ax = plt.subplots(figsize=(50, 50))
# ax.figure(figsize=(50, 50))

# ax.axis([0, 100, 0, 50])  # Set the axis limits of the plot
ax.set_aspect('equal')
# Display a message to the user to start clicking the mouse
plt.tight_layout(pad=15, h_pad=None, w_pad=None, rect=None)

ax.xaxis.set_ticks(range(0, 200, 5))
ax.yaxis.set_ticks(range(0, 100, 5))
ax.grid(color='green', linestyle='-', linewidth=0.5)


print("Click the plot to get the coordinates")

# Get the coordinates of mouse clicks
points = fig.ginput(n=8, show_clicks=True, mouse_add=1)  # n=3 means we want 3 points, timeout=30 means the function will wait for 30 seconds

# Print the coordinates of the mouse clicks
for point in points:
    print(point)
