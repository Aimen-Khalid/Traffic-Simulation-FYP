import math
import dcel
import car
import matplotlib.pyplot as plt
import keyboard
from matplotlib.animation import FuncAnimation
# import delaunay

# Set the figure size and create the axis
fig, ax = plt.subplots(figsize=(200, 100))
ax.set_aspect('equal')

# Set the axis limits and add the grid lines
ax.axis([0, 50, 0, 50])
ax.xaxis.set_ticks(range(0, 200, 5))
ax.yaxis.set_ticks(range(0, 100, 5))
ax.grid(color='green', linestyle='-', linewidth=0.5)

vehicle = car.Vehicle(length=10, width=5, speed_limit=0, acc_limit=0, centroid=car.Point(25, 25),
                      angle=5, v=car.Point(0, 0), a=car.Point(0, 0))


def get_face_vertices(face):
    hedge = face.outer_component
    face_vertices = [[hedge.origin.x, hedge.origin.y]]
    hedge = hedge.next
    while hedge != face.outer_component:
        face_vertices.append((hedge.origin.x, hedge.origin.y))
        hedge = hedge.next
    return face_vertices


def draw_rectangle(p):
    x, y = p[0], p[1]
    plt.plot(x, y)


def draw_vehicle(vehicle):
    p = vehicle.get_xy_lists()
    draw_rectangle(p)


def show_dcel(my_dcel):
    p = vehicle.get_xy_lists()
    draw_rectangle(p)
    # draw_car_perpendicular_line(vehicle)
    for face in my_dcel.faces:
        vertices = get_face_vertices(face)
        x = [x for x, y in vertices]
        y = [y for x, y in vertices]
        plt.fill(x, y, color=face.fill_color)
        for vertex in vertices:
            plt.scatter(vertex[0], vertex[1])
    draw_vehicle(vehicle)
    plt.show()


def simulate(my_dcel, vehicles, frames, fn):
    current_face = my_dcel.faces[0]
    vehicle.error_point, vehicle.error = get_error(vehicle, current_face)
    x_list, y_list = [], []
    for V in vehicles:
        # get the x and y coordinates for the current vehicle
        x, y = V.get_xy_lists()
        # append the x and y coordinates to the lists
        x_list.append(x)
        y_list.append(y)

    lines = []
    texts = []
    for j in range(len(vehicles)):
        line, = ax.plot(x_list[j], y_list[j])
        text = ax.text(vehicles[j].centroid.x, vehicles[j].centroid.y, vehicles[j].velocity.norm())
        text.set_size(8)
        lines.append(line)  # storing all 2d lines (rectangles) in a list
        texts.append(text)

    def init():
        lines[0].set_data(x_list[0], y_list[0])
        texts[0].set_text(f'{vehicles[0].velocity.norm()} km/h')
        texts[0].set_position((vehicles[0].centroid.x, vehicles[0].centroid.y))
        return lines + texts

    def animate(i):
        for count, V in enumerate(vehicles):
            # update the state variables for each vehicle
            V.update_state_vars()
            # get the updated x and y coordinates for the current vehicle
            x, y = V.get_xy_lists()
            # update the x and y lists with the updated coordinates
            x_list[count] = x
            y_list[count] = y
            # leaving a trail behind the vehicle
            # if i%5==0:
            #   axis.plot(x[3], y[3], marker=".",markersize=1,markerfacecolor="black")

        # update all lines with the updated x and y lists
        for j in range(len(vehicles)):
            lines[j].set_data(x_list[j], y_list[j])
            texts[j].set_text(f'{int(vehicles[j].velocity.norm())} km/h')
            # texts[j].set_text(f'{Vs[j].centroid.x} {Vs[j].centroid.y}')
            x, y = vehicles[j].centroid.x, vehicles[j].centroid.y
            texts[j].set_position((x, y))
        # update axis limit to keep the vehicles in view
        vehicle_xs = [element for sublist in x_list for element in sublist]
        vehicle_ys = [element for sublist in y_list for element in sublist]
        ax.set_xlim(min(vehicle_xs) - 5, max(vehicle_xs) + 5)
        ax.set_ylim(min(vehicle_ys) - 5, max(vehicle_ys) + 5)
        return lines + texts

    anim = FuncAnimation(fig, animate, init_func=init,
                         frames=frames, blit=True)

    anim.save(fn, writer='ffmpeg', fps=30)


def get_vertices_list(vertices):
    return [(vertex.x, vertex.y) for vertex in vertices]


def write_vertices_to_file(vertices):
    with open("vertices1.txt", "w") as file:
        for vertex in vertices:
            for coordinate in vertex:
                file.write(str(coordinate) + "\t")
            file.write("\n")


def load_vertices_from_file():
    vertices = []
    with open("vertices1.txt", "r") as file:
        for line in file:
            vertex = tuple(int(x) for x in line.strip().split("\t"))
            vertices.append(vertex)
    return vertices


def write_segments_to_file(segments):
    with open("segments1.txt", "w") as file:
        for segment in segments:
            for vertices in segment:
                for vertex in vertices:
                    file.write(str(vertex) + "\t")
            file.write("\n")


def load_segments_from_file():
    segments = []
    with open("segments1.txt", "r") as file:
        for line in file:
            segment = []
            vertices = [int(x) for x in line.strip().split("\t")]
            segment.append((vertices[0], vertices[1]))
            segment.append((vertices[2], vertices[3]))
            segments.append(segment)
    return segments


def get_car_perpendicular_line(vehicle):
    x, y = vehicle.get_xy_lists()
    x1, x2 = x[0], x[1]
    y1, y2 = y[0], y[1]

    return [(x1, y1), (x2, y2)]


# def draw_car_perpendicular_line(vehicle):
#     segment = get_car_perpendicular_line(vehicle)
#     color = (211, 211, 211)  # light grey
#     x1 = 0
#     y1 = get_y_at_x((segment[0][0], segment[0][1]), (segment[1][0], segment[1][1]), x1)
#     x2 = screen_width
#     y2 = get_y_at_x((segment[0][0], segment[0][1]), (segment[1][0], segment[1][1]), x2)
#     pygame.draw.line(screen, color, (x1, y1), (x2, y2))


def get_vertices_and_segments():
    prev_x = 0
    prev_y = 0

    vertices = []
    segments = []
    first = True
    run = True
    # Snap the points to the nearest grid point
    grid_spacing = 5
    while run:
        point = fig.ginput(n=1, show_clicks=True, mouse_add=1)
        x = round(point[0][0] / grid_spacing) * grid_spacing
        y = round(point[0][1] / grid_spacing) * grid_spacing
        if (x, y) not in vertices:
            vertices.append((x, y))
        # Print the snapped coordinates of the mouse clicks
        # print(list(zip(x, y)))

        # Plot the snapped points on the figure
        if not first:
            plt.plot([x, prev_x], [y, prev_y], 'o-')
            segment = [(prev_x, prev_y), (x, y)]
            if prev_x != x or prev_y != y:
                segments.append(segment)
        else:
            plt.plot(x, y, 'o-')
        # Display the resulting plot
        plt.draw()
        prev_x = x
        prev_y = y
        first = bool(keyboard.is_pressed('space'))
        if keyboard.is_pressed('d'):
            run = False
    return vertices, segments


def main():
    vertices, segments = get_vertices_and_segments()
    dcel_obj = dcel.Dcel()
    dcel_obj.build_dcel(vertices, segments)
    show_dcel(dcel_obj)
    write_vertices_to_file(vertices)
    write_segments_to_file(segments)


def get_m_and_b(segment):
    x1, y1 = segment[0]
    x2, y2 = segment[1]

    if x2-x1 == 0:
        m = "inf"
        b = 0
    else:
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
    return m, b


def get_y_at_x(origin, destination, x):
    m, b = get_m_and_b([origin, destination])
    return m*x + b


def get_intersection_point(segment1, segment2):
    m1, b1 = get_m_and_b(segment1)
    m2, b2 = get_m_and_b(segment2)
    if m1 != "inf" and m2 != "inf" and m1 - m2 == 0:
        return [None]
    if m1 == "inf":     # vertical line
        x = segment1[0][0]
        y = get_y_at_x((segment2[0][0], segment2[0][1]), (segment2[1][0], segment2[1][1]), x)
    elif m2 == "inf":   # vertical line
        x = segment2[0][0]
        # get x at y
        y = get_y_at_x((segment1[0][0], segment1[0][1]), (segment1[1][0], segment1[1][1]), x)
    elif m1 == 0:         # horizontal line y = c
        y = segment1[0][1]
        x = get_y_at_x((segment2[0][1], segment2[0][0]), (segment2[1][1], segment2[1][0]), y)
    elif m2 == 0:
        y = segment2[0][1]
        x = get_y_at_x((segment1[0][1], segment1[0][0]), (segment1[1][1], segment1[1][0]), y)
    else:
        x = (b2 - b1) / (m1 - m2)
        y = m1*x + b1
    return [x, y]


def x_lies_between(x1, x2, x):
    x_min = min(x1, x2)
    x_max = max(x1, x2)
    return x_min <= x <= x_max


def get_mid_point(segment):
    return (segment[0][0] + segment[1][0])/2, (segment[0][1] + segment[1][1])/2


def get_point_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)


def get_error(vehicle, face):
    half_edge = face.outer_component
    intersection_points = []
    first_iteration = True
    while first_iteration or half_edge is not face.outer_component:
        first_iteration = False
        segment1 = [(half_edge.origin.x, half_edge.origin.y), (half_edge.destination.x, half_edge.destination.y)]
        segment2 = get_car_perpendicular_line(vehicle)
        intersection_point = get_intersection_point(segment1, segment2)
        if (
            intersection_point[0] is not None
            and x_lies_between(
                x1=half_edge.origin.x,
                x2=half_edge.destination.x,
                x=intersection_point[0],
            )
            and x_lies_between(
                x1=half_edge.origin.y,
                x2=half_edge.destination.y,
                x=intersection_point[1],
            )
        ):
            intersection_points.append((intersection_point[0], intersection_point[1]))

        half_edge = half_edge.next
    if len(intersection_points) < 2:
        return (0, 0), 0
    mid_point_x, mid_point_y = get_mid_point([(intersection_points[0][0], intersection_points[0][1]),
                                              (intersection_points[1][0], intersection_points[1][1])])
    segment = get_car_perpendicular_line(vehicle)
    vehicle_x = (segment[0][0] + segment[1][0])/2
    vehicle_y = (segment[0][1] + segment[1][1]) / 2
    return (mid_point_x, mid_point_y), \
        get_point_distance((mid_point_x, mid_point_y), (vehicle_x, vehicle_y))


def build_dcel_from_file():
    vertices = load_vertices_from_file()
    segments = load_segments_from_file()
    my_dcel = dcel.Dcel()
    my_dcel.build_dcel(vertices, segments)
    # show_dcel(my_dcel)
    return my_dcel


main()
# my_dcel = build_dcel_from_file()
# show_dcel(my_dcel)
# Vs = [vehicle]
# simulate(my_dcel, Vs, frames=500, fn="simulation6.mp4")

