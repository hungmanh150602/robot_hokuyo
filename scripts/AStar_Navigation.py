#!/usr/bin/env python3
import numpy as np
import rclpy
import matplotlib.pyplot as plt

from rclpy.node import Node

from load_map import load_map

class Planner(Node):
    def __init__(self):
        super().__init__('Planner')

        # -------------------------------
        # Load map + robot state
        # -------------------------------
        self.map_array, self.resolution, self.origin = load_map(
            "/home/hungubuntu/ros2_workspace/src/robot_hokuyo/maps/my_map_save.yaml"
        )

        self.cell_size = 0.3  # mét

        self.display_map()

    def display_map(self):
        # Kích thước map gốc
        height, width = self.map_array.shape

        # Tính tọa độ thực
        x_min = self.origin[0]
        x_max = self.origin[0] + width * self.resolution

        y_min = self.origin[1]
        y_max = self.origin[1] + height * self.resolution

        # ==========================
        # TẠO MAP MỚI 0.3m
        # ==========================
        map_width_m = width * self.resolution
        map_height_m = height * self.resolution

        new_cols = int(np.ceil(map_width_m / self.cell_size))
        new_rows = int(np.ceil(map_height_m / self.cell_size))

        new_map = np.zeros((new_rows, new_cols))

        for r in range(new_rows):
            for c in range(new_cols):

                x0 = c * self.cell_size
                x1 = min((c + 1) * self.cell_size, map_width_m)

                y0 = r * self.cell_size
                y1 = min((r + 1) * self.cell_size, map_height_m)

                # Chuyển sang pixel của map gốc
                px0 = int(x0 / self.resolution)
                px1 = int(x1 / self.resolution)

                py0 = int(y0 / self.resolution)
                py1 = int(y1 / self.resolution)

                block = self.map_array[py0:py1, px0:px1]

                if block.size > 0:
                    # Nếu có vật cản thì ô mới là vật cản
                    if np.any(block < 100):
                        new_map[r, c] = 1

        # ==========================
        # VẼ
        # ==========================
        fig, axs = plt.subplots(1, 2, figsize=(16, 8))

        # Map gốc
        axs[0].imshow(
            self.map_array,
            cmap='gray',
            origin='lower',
            extent=[x_min, x_max, y_min, y_max]
        )

        axs[0].set_title("Original Map")
        axs[0].set_xlabel("X (m)")
        axs[0].set_ylabel("Y (m)")
        axs[0].grid(True)

        # Map 0.3m
        axs[1].imshow(
            new_map,
            cmap='gray_r',
            origin='lower',
            extent=[x_min, x_max, y_min, y_max]
        )

        axs[1].set_title("0.3m Grid Map")
        axs[1].set_xlabel("X (m)")
        axs[1].set_ylabel("Y (m)")

        # Vẽ lưới 0.3m
        for x in np.arange(x_min, x_max, self.cell_size):
            axs[1].axvline(x, color='blue', linewidth=0.5)

        for y in np.arange(y_min, y_max, self.cell_size):
            axs[1].axhline(y, color='blue', linewidth=0.5)

        plt.tight_layout()
        plt.show()


# --------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------
def main():
    rclpy.init()
    planner = Planner()
    rclpy.spin(planner)


if __name__ == "__main__":
    main()