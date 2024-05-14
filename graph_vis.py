import matplotlib.pyplot as plt
import os
import numpy as np
from scipy.interpolate import interp1d

def read_and_plot_data(*filenames):
    plt.figure(figsize=(8, 8))  # Set figure size to create a square
    
    for filename in filenames:
        iterations = [0]
        cost = [1000]

        with open(filename, 'r') as file:
            for line in file:
                parts = line.split(';')
                try:
                    iter_part = parts[1].strip().split(': ')[1]
                    cost_part = parts[0].split(': ')[2]
                    iterations.append(int(iter_part))
                    cost.append(float(cost_part))
                except (IndexError, ValueError) as e:
                    print(f"Invalid data format in line: {line.strip()}")

        if iterations and cost:  # Check if any valid data was collected
            # Sort data by iterations
            sorted_data = sorted(zip(iterations, cost), key=lambda x: x[0])
            iterations, cost = zip(*sorted_data)

            # Linear interpolation
            f = interp1d(iterations, cost)

            # New x values for smoother graph
            new_iterations = np.linspace(min(iterations), max(iterations), 1000)
            new_cost = f(new_iterations)

            plt.plot(new_iterations, new_cost, linestyle='-', linewidth=5, alpha=0.5, label=os.path.splitext(os.path.basename(filename))[0])

    fontsize_ticks=20
    fontsize_labels=25

    # plt.xlabel('Iterations', fontsize=fontsize_labels)
    # plt.ylabel('Cost', fontsize=fontsize_labels) 
    plt.grid(True)
    plt.xticks(range(0, 10001, 2000), fontsize=fontsize_ticks)  # Set x-axis tick marks every 100
    plt.yticks(range(0, 500, 10), fontsize=fontsize_ticks)  # Set y-axis tick marks every 50
    # plt.xlim(0,1000)
    plt.ylim(390, 430)
    # plt.gca().set_aspect('equal', adjustable='box')  # Set aspect ratio to be equal
    plt.legend(fontsize=fontsize_ticks)  # Show legend for all graphs
    plt.savefig('/home/tsoyarty/Desktop/Bc_work/main/graphs/Maze_narrow/graph.pdf', bbox_inches='tight', pad_inches=0)
    plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap5/graph_narrow_20pt.pdf', bbox_inches='tight', pad_inches=0)
    plt.show()

# Example usage:
read_and_plot_data('graphs/Maze_narrow/RRT*_ML.txt','graphs/Maze_narrow/RRT*.txt','graphs/Maze_narrow/InformedRRT*.txt','graphs/Maze_narrow/RRT#.txt', 'graphs/Maze_narrow/RRTXstatic.txt')
 
# def read_and_plot_data(filename):
#     iterations = []
#     cost = []

#     with open(filename, 'r') as file:
#         for line in file:
#             parts = line.split(';')
#             try:
#                 iter_part = parts[1].strip().split(': ')[1]
#                 cost_part = parts[0].split(': ')[2]
#                 iterations.append(int(iter_part))
#                 cost.append(float(cost_part))
#             except (IndexError, ValueError) as e:
#                 print(f"Invalid data format in line: {line.strip()}")

#     if iterations and cost:  # Check if any valid data was collected
#         plt.plot(iterations, cost, marker='o', linestyle='-')
#         plt.xlabel('Iterations')
#         plt.ylabel('Cost')
#         plt.title('Cost vs Iterations')
#         plt.grid(True)
#         plt.xticks(range(0, max(iterations)+100, 50))  # Set x-axis tick marks every 100
#         plt.yticks(range(0, int(max(cost))+100, 50))  # Set y-axis tick marks every 50
#         plt.gca().set_aspect('equal', adjustable='box')  # Set aspect ratio to be equal
#         plt.show()
#     else:
#         print("No valid data found in the file.")

# # Example usage:
# read_and_plot_data('RRTstar_graph.txt') 
