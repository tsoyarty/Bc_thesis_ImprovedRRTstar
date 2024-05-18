import matplotlib.pyplot as plt
import numpy as np  
from my_utils import fill_polygon, read_maze_file
 
start_point, end_point, polygons = read_maze_file('Maze_clutter')

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
 
goalRegion = plt.Circle((end_point[0], end_point[1]), 5, color="red", fill = False)
plt.gca().add_patch(goalRegion) 
# plt.text(75, 10, r'$\mathcal{C}_{\mathrm{free}}$', fontsize=12, ha='center', va='top')

# Plot the path
# plt.plot(x_values, y_values, 'bo-')
plt.plot(start_point[0], start_point[1], 'go')
plt.plot(end_point[0], end_point[1], 'ro') 

plt.plot(x_values, y_values, 'o-', color='cyan', linewidth = 3)
# plt.plot(x_values[0], y_values[0], 'go', label = r'$q_{\mathrm{start}}$')
# plt.plot(x_values[-1], y_values[-1], 'ro', label = r'$q_{\mathrm{goal}}$') 
# plt.text(x_values[0], y_values[0], r'$q_{\mathrm{start}}$', fontsize=12, ha='right', va='bottom')
# plt.text(x_values[-1], y_values[-1], "q_goal (goal region)", fontsize=12, ha='right', va='bottom') 
# plt.legend()

# Remove axes numbers (ticks)
# plt.xticks([])
# plt.yticks([])
plt.xlim(0,grid.shape[0]-1)
plt.ylim(0,grid.shape[1]-1)
# Set axis equal to ensure the circle looks like a circle 
# plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap5/Maze_clutter_final_solution_RRTXstatic.pdf', bbox_inches='tight', pad_inches=0)
# plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap5/Maze_clutter_first_solution_RRTXstatic.pdf', bbox_inches='tight', pad_inches=0)
# plt.savefig('/home/tsoyarty/Desktop/Bc_work/main/graphs/Maze_clutter/Maze_clutter_final_solution_RRTstar.pdf', bbox_inches='tight', pad_inches=0)
    
plt.show()
