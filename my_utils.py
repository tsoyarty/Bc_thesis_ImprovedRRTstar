import numpy as np


#-----------------HELP FUNCTIONS----------------------

def fill_polygon(grid, polygon):
    min_x = int(min(polygon[:, 0]))
    max_x = int(max(polygon[:, 0]))
    min_y = int(min(polygon[:, 1]))
    max_y = int(max(polygon[:, 1]))

    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            if is_point_inside_polygon(x, y, polygon):
                grid[y][x] = 1

def is_point_inside_polygon(x, y, polygon):
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def read_maze_file(filename):
    Obstacles = []
    with open(filename, 'r') as file:
        start_point = np.array(list(map(int, file.readline().split(':')[1].strip().split(','))))
        end_point = np.array(list(map(int, file.readline().split(':')[1].strip().split(','))))
        for line in file:
            coords = line.strip().split()
            polygon = np.array([[int(coord.split(',')[0]), int(coord.split(',')[1])] for coord in coords])
            # print(polygon)
            Obstacles.append(polygon)
    return start_point, end_point, Obstacles 

