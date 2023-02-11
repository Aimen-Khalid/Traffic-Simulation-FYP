import pygame
import math
import dcel
import car
import matplotlib.pyplot as plt
import keyboard
# import delaunay

pygame.init()
screen_width = 1500
screen_height = 750
screen = pygame.display.set_mode((screen_width, screen_height))
clock = pygame.time.Clock()

# Set the figure size and create the axis
fig, ax = plt.subplots(figsize=(50, 50))
ax.set_aspect('equal')

# Set the axis limits and add the grid lines
ax.axis([0, 50, 0, 50])
ax.xaxis.set_ticks(range(0, 50, 5))
ax.yaxis.set_ticks(range(0, 50, 5))
ax.grid(color='green', linestyle='-', linewidth=0.5)

vehicle = car.Vehicle(length=100, width=50, speed_limit=0, acc_limit=0, centroid=car.Point(345, 85),
                      angle=5, v=car.Point(0, 0), a=car.Point(0, 0))


def draw_grid(width, rows, surface):
    size_between = width // rows

    for i in range(0, width, size_between):
        x, y = i, i
        color = (211, 211, 211)  # light grey
        pygame.draw.line(surface, color, (x, 0), (x, width))
        pygame.draw.line(surface, color, (0, y), (width, y))


class Click:
    def _init_(self):
        self.x = None
        self.y = None

    def set_coordinates(self, size_between):
        x, y = pygame.mouse.get_pos()

        mouse_x = x
        mouse_y = y

        offset_x = x % size_between
        offset_y = y % size_between

        if offset_x > (size_between / 2):
            if offset_y > (size_between / 2):  # bottom right corner
                mouse_x += size_between - offset_x
                mouse_y += size_between - offset_y

            else:  # top right corner
                mouse_x += size_between - offset_x
                mouse_y -= offset_y

        elif offset_y < (size_between / 2):  # top left corner
            mouse_x -= offset_x
            mouse_y -= offset_y

        else:  # bottom right corner
            mouse_x -= offset_x
            mouse_y += size_between - offset_y

        self.x, self.y = mouse_x, mouse_y


def draw_vertex(x, y, radius=3):
    color = (0, 0, 0)
    pygame.draw.circle(screen, color, (x, y), radius)
    font = pygame.font.Font(None, 25)
    text = font.render(f" {str(x)}, {str(y)}", True, (0, 0, 255))
    screen.blit(text, (x, y))


def draw_edge(x1, y1, x2, y2):
    pygame.draw.line(screen, (255, 0, 0), (x1, y1), (x2, y2))


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
    rectangle = [(x[0], y[0]), (x[1], y[1]), (x[2], y[2]), (x[3], y[3])]
    pygame.draw.polygon(screen, (200, 100, 60), rectangle, 0)


def show_dcel(my_dcel):
    current_face = my_dcel.faces[0]
    run = True

    p = vehicle.get_xy_lists()
    # draw_rectangle(p)
    # draw_car_perpendicular_line(vehicle)
    for face in my_dcel.faces:
        vertices = get_face_vertices(face)
        x = [x for x, y in vertices]
        y = [y for x, y in vertices]
        plt.fill(x, y, color=face.fill_color)
        for vertex in vertices:
            plt.scatter(vertex[0], vertex[1])
    plt.show()
    # pygame.display.flip()
    # i = 0
    # while run:
    #     clock.tick(60)
    #     p = vehicle.get_xy_lists()
    #     vehicle.error_point, vehicle.error = get_error(vehicle, current_face)
    #     vehicle.error_point = car.Point(vehicle.error_point[0], vehicle.error_point[1])
    #     draw_rectangle(p)
    #     pygame.draw.circle(screen, (15, 15, 15), (vehicle.error_point.x, vehicle.error_point.y), 2)
    #     if i % 20 == 0 and i > 50:
    #         draw_car_perpendicular_line(vehicle)
    #     vehicle.update_state_vars()
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             run = False
    #     pygame.display.flip()
    #     i += 1


# def triangulate_face(face):
#     vertices = get_face_vertices(face)
#     triangles = delaunay(vertices)
#     triangles_vertices_indices = list(triangles.simplices)
#     segments = []
#     for triangle in triangles_vertices_indices:
#         v1, v2, v3 = triangle
#         segments.extend(
#             (
#                 [vertices[v1], vertices[v2]],
#                 [vertices[v2], vertices[v3]],
#                 [vertices[v3], vertices[v1]],
#             )
#         )
#     return segments


def get_vertices_list(vertices):
    return [(vertex.x, vertex.y) for vertex in vertices]


# def triangulate_dcel(dcel_to_triangulate):
#     segments = []
#     for face in dcel_to_triangulate.faces:
#         segments.extend(triangulate_face(face))
#     vertices = get_vertices_list(dcel_to_triangulate.get_vertices())
#
#     my_dcel = dcel.Dcel()
#     my_dcel.build_dcel(vertices, segments)
#
#     my_dcel.show_dcel()


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


def draw_car_perpendicular_line(vehicle):
    segment = get_car_perpendicular_line(vehicle)
    color = (211, 211, 211)  # light grey
    x1 = 0
    y1 = get_y_at_x((segment[0][0], segment[0][1]), (segment[1][0], segment[1][1]), x1)
    x2 = screen_width
    y2 = get_y_at_x((segment[0][0], segment[0][1]), (segment[1][0], segment[1][1]), x2)
    pygame.draw.line(screen, color, (x1, y1), (x2, y2))


def get_vertices_and_segments():
    # Get the coordinates of the mouse clicks
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
            if not (prev_x == x and prev_y == y):
                segments.append(segment)
        else:
            plt.plot(x, y, 'o-')
        # Display the resulting plot
        plt.draw()
        prev_x = x
        prev_y = y
        first = False
        if keyboard.is_pressed('space'):
            first = True
        if keyboard.is_pressed('d'):
            run = False
    return vertices, segments


def get_vertices_and_segments1():
    click = Click()
    run = True
    vertices = []
    segments = []
    screen.fill((255, 255, 255))

    num_of_cols = 100
    draw_grid(screen.get_width(), num_of_cols, screen)
    escape_pressed = False
    while run:
        clock.tick(60)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    escape_pressed = True
                if event.key == pygame.K_RETURN:
                    run = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                click.set_coordinates(screen.get_width() // num_of_cols)
                # vertices.append((click.x, screen.get_height() - click.y))
                vertices.append((click.x, click.y))
                # draw_vertex(vertices[-1][0], (vertices[-1][1] - screen.get_height()) * -1)
                draw_vertex(vertices[-1][0], vertices[-1][1])
                if not escape_pressed and len(vertices) > 1:
                    segment = [vertices[-1], vertices[-2]]
                    # draw_edge(vertices[-1][0], (vertices[-1][1] - screen.get_height()) * -1, vertices[-2][0],
                    #           (vertices[-2][1] - screen.get_height()) * -1)
                    draw_edge(vertices[-1][0], vertices[-1][1], vertices[-2][0],
                              vertices[-2][1])
                    segments.append(segment)
                if escape_pressed:
                    escape_pressed = False

        pygame.display.flip()
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
#
# face = my_dcel.faces[0]
# print(get_error(vehicle, face))
# show_dcel(my_dcel)

# vertices, segments = get_vertices_and_segments()
#
# print(vertices)
# print(segments)