import yaml
import numpy as np
from PIL import Image
import os


def load_map(yaml_file, grid_size = 0.3):
#======================================================================================================================================================#
    FREE = 0
    OBSTACLE = 1
    UNKNOWN = 2
#======================================================================================================================================================#
    # Đường dẫn file map.yaml
    # yaml_file = "/home/hungubuntu/ros2_workspace/src/my_first_pkg/maps/my_map_save.yaml"

    # Đọc file YAML
    with open(yaml_file, 'r') as file:
        map_config = yaml.safe_load(file)

    # Lấy thông tin từ YAML
    image_path = map_config['image']
    resolution = map_config['resolution']
    origin = map_config['origin']

    # Nếu image là đường dẫn tương đối
    yaml_dir = os.path.dirname(yaml_file)
    image_full_path = os.path.join(yaml_dir, image_path)

    # Đọc ảnh map
    map_image = Image.open(image_full_path)

    # Chuyển sang numpy array
    map_array = np.array(map_image)

    # Nếu ảnh nhiều kênh màu -> grayscale
    if len(map_array.shape) == 3:
        map_array = map_array[:, :, 0]

    map_array = np.flipud(map_array)
#======================================================================================================================================================#
#======================================================================================================================================================#
    # Convert occupancy
    cell_pixels = int(grid_size / resolution)

    grid_map = np.zeros_like(map_array)

    grid_map[map_array < 50] = OBSTACLE
    grid_map[(map_array >= 50) & (map_array < 250)] = UNKNOWN
    grid_map[map_array >= 250] = FREE

    # A* grid params
    h, w = grid_map.shape

    grid_h = h // cell_pixels
    grid_w = w // cell_pixels

    astar_grid = np.zeros((grid_h, grid_w), dtype=np.uint8)

    for gy in range(grid_h):
        for gx in range(grid_w):
            y0 = gy * cell_pixels
            y1 = y0 + cell_pixels

            x0 = gx * cell_pixels
            x1 = x0 + cell_pixels

            block = grid_map[y0:y1, x0:x1]
            obstacle_ratio = np.mean(block == OBSTACLE)
            unknown_ratio = np.mean(block == UNKNOWN)

            if obstacle_ratio > 0.15:
                astar_grid[gy, gx] = 1
            elif unknown_ratio > 0.8:
                astar_grid[gy, gx] = 1
            else:
                astar_grid[gy, gx] = 0
#======================================================================================================================================================#

    return map_array, astar_grid, resolution, origin
