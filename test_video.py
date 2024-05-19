import matplotlib.pyplot as plt
import numpy as np
import os
import imageio

# Example data
x = np.linspace(0, 10, 100)

# Create a directory to save the images
if not os.path.exists('plots'):
    os.makedirs('plots')

# Generate and save plots
for i in range(10):
    y = np.sin(x + i * 0.1)
    plt.figure()
    plt.plot(x, y)
    plt.title(f'Plot {i}')
    plt.savefig(f'plots/plot_{i:03d}.png')
    plt.close()

# Get list of saved image files
image_files = [f'plots/plot_{i:03d}.png' for i in range(10)]

# Create a video
with imageio.get_writer('visualization.mp4', fps=2) as writer:  # fps can be adjusted as needed
    for filename in image_files:
        image = imageio.imread(filename)
        writer.append_data(image)
