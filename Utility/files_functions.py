def write_vertices_to_file(vertices, area_name):
    with open(f'{area_name}_vertices.txt', "w") as file:
        for vertex in vertices:
            for coordinate in vertex:
                file.write(str(coordinate) + "\t")
            file.write("\n")


def write_obstacles_to_file(vertices, area_name):
    with open(f'{area_name}_obstacles_positions.txt', "w") as file:
        for vertex in vertices:
            for coordinate in vertex:
                file.write(str(coordinate) + "\t")
            file.write("\n")


def load_vertices_from_file(area_name):
    file_name = f'{area_name}_vertices.txt'
    vertices = []
    try:
        with open(file_name, "r") as file:
            for line in file:
                vertex = tuple(float(x) for x in line.strip().split('\t'))
                vertices.append(vertex)
    except FileNotFoundError as e:
        raise FileNotFoundError(
            f'{file_name} does not exist. Create vertices file and name as areaname_vertices.txt'
        ) from e
    return vertices


def load_obstacles_from_file(area_name):
    file_name = f'{area_name}_obstacles_positions.txt'
    vertices = []
    try:
        with open(file_name, "r") as file:
            for line in file:
                vertex = tuple(float(x) for x in line.strip().split('\t'))
                vertices.append(vertex)
    except FileNotFoundError as e:
        raise FileNotFoundError(
            f'{file_name} does not exist. Create vertices file and name as areaname_vertices.txt'
        ) from e
    return vertices


def write_segments_to_file(segments, area_name):
    with open(f'{area_name}_segments.txt', "w") as file:
        for segment in segments:
            for vertices in segment:
                for vertex in vertices:
                    file.write(str(vertex) + "\t")
            file.write("\n")


def load_segments_from_file(area_name):
    segments = []
    with open(f'{area_name}_segments.txt', "r") as file:
        for line in file:
            vertices = [float(x) for x in line.strip().split("\t")]
            segment = [(vertices[0], vertices[1]), (vertices[2], vertices[3])]
            segments.append(segment)
    return segments









