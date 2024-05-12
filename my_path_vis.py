import matplotlib.pyplot as plt
import numpy as np  

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

def read_polygons_from_file(filename):
    polygons = []
    with open(filename, 'r') as file:
        start_point = np.array(list(map(int, file.readline().split(':')[1].strip().split(','))))
        end_point = np.array(list(map(int, file.readline().split(':')[1].strip().split(','))))
        for line in file:
            coords = line.strip().split()
            polygon = np.array([[int(coord.split(',')[0]), int(coord.split(',')[1])] for coord in coords])
            polygons.append(polygon)
    return start_point, end_point, polygons

start_point, end_point, polygons = read_polygons_from_file('Maze_U')

rows, cols = 150, 150
grid = np.zeros((rows, cols))    

for poly in polygons:
    fill_polygon(grid, poly)

fig = plt.figure("OMPL")
plt.imshow(grid, cmap='binary') 

# Obstacle = plt.Circle((75,75),30, color="black", fill = True) 
# ax = fig.gca()
# ax.add_patch(Obstacle)

file_path = "main.txt"
with open(file_path, 'r') as file:
    lines = file.readlines()

x_values = []
y_values = []

for line in lines:
    values = line.strip().split()
    if values:
        x_values.append(float(values[0]))
        y_values.append(float(values[1])) 

# Enable LaTeX rendering
plt.rc('text', usetex=True)
 
goalRegion = plt.Circle((x_values[-1], y_values[-1]),5, color="red", fill = False)
plt.gca().add_patch(goalRegion) 
# plt.text(75, 10, r'$\mathcal{C}_{\mathrm{free}}$', fontsize=12, ha='center', va='top')

# Plot the path
plt.plot(x_values, y_values, 'bo-')
plt.plot(x_values[0], y_values[0], 'go')
plt.plot(x_values[-1], y_values[-1], 'ro') 

# plt.plot(x_values, y_values, 'bo-', label=r'$\mathrm{path}$')
# plt.plot(x_values[0], y_values[0], 'go', label = r'$q_{\mathrm{start}}$')
# plt.plot(x_values[-1], y_values[-1], 'ro', label = r'$q_{\mathrm{goal}}$') 
# plt.text(x_values[0], y_values[0], r'$q_{\mathrm{start}}$', fontsize=12, ha='right', va='bottom')
# plt.text(x_values[-1], y_values[-1], "q_goal (goal region)", fontsize=12, ha='right', va='bottom') 
# plt.legend()

# Remove axes numbers (ticks)
plt.xticks([])
plt.yticks([])
plt.xlim(0,150)
plt.ylim(0,150)
# Set axis equal to ensure the circle looks like a circle 
# plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap2/ConfigSpace.eps', bbox_inches='tight', pad_inches=0)
plt.show()
