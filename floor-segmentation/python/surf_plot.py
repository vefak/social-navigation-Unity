import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from mpl_toolkits.mplot3d import Axes3D
import json

root = '.'

with open(f'{root}/json/config.json') as json_data:
    data = json.load(json_data)
    json_data.close()
    path = f'{root}/output/snapshots'
    img_name = 'dynamic_potential'

    # Step 1: Load the image
    image_path = f'{path}/{img_name}.png'  # Replace with your image path
    image = Image.open(image_path).convert('L')  # Convert to grayscale

    # Step 2: Convert the image to a NumPy array for easier processing
    image_data = np.max(image) - np.array(image)

    # Step 3: Generate X, Y coordinates for the surface plot
    x = np.arange(image_data.shape[1])
    y = np.arange(image_data.shape[0])
    X, Y = np.meshgrid(x, y)

    # Step 4: Create a surface plot using Matplotlib
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    surface = ax.plot_surface(X, Y, image_data, cmap='viridis', edgecolor='none')

    # Add color bar to indicate intensity
    fig.colorbar(surface, ax=ax, shrink=0.5, aspect=5)

    # Step 5: Set plot labels and display the plot
    ax.set_title('3D Surface Plot of Potential Field')
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Potential')

    plt.savefig(f'{path}/{img_name}_mesh.png')