import yaml
import numpy as np
from PIL import Image
import os


def load_map(yaml_file):

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

    return map_array, resolution, origin
