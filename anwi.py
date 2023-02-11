import matplotlib.pyplot as plt
import keyboard


# Set the figure size and create the axis
fig, ax = plt.subplots(figsize=(50, 50))
ax.set_aspect('equal')

# Set the axis limits and add the grid lines
ax.axis([0, 50, 0, 50])
ax.xaxis.set_ticks(range(0, 50, 5))
ax.yaxis.set_ticks(range(0, 50, 5))
ax.grid(color='green', linestyle='-', linewidth=0.5)

# Print a message for the user
print("Click the plot to get the coordinates")

# Get the coordinates of the mouse clicks
prev_x = 0
prev_y = 0
first = True
while True:
    points = fig.ginput(n=1, show_clicks=True, mouse_add=1)
    # Snap the points to the nearest grid point
    grid_spacing = 5
    x = [round(point[0] / grid_spacing) * grid_spacing for point in points]
    y = [round(point[1] / grid_spacing) * grid_spacing for point in points]

    # Print the snapped coordinates of the mouse clicks
    # print(list(zip(x, y)))

    # Plot the snapped points on the figure
    if not first:
        plt.plot([x, prev_x], [y, prev_y], 'o-')
    else:
        plt.plot(x, y, 'o-')
    # Display the resulting plot
    plt.draw()
    prev_x = x
    prev_y = y
    first = False
    if keyboard.is_pressed('space'):
        first = True
        print("escape pressed")
