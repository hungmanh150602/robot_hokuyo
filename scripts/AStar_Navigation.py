#!/usr/bin/env python3
import numpy as np
import rclpy
import matplotlib.pyplot as plt

from rclpy.node import Node

from load_map import load_map
from a_star import AStar

class Planner(Node):
    def __init__(self):
        super().__init__('Planner')

        # Load map + robot state
        self.grid_size = 0.3

        self.map_array, self.grid_map, self.resolution, self.origin = load_map(
            "/home/hungubuntu/ros2_workspace/src/robot_hokuyo/maps/my_map_save.yaml",
            self.grid_size
        )

        # Goal (in world coordinates)
        self.start_world = (0.0, 0.0)
        self.goal_world = (3.5, 1.0)

        # A* path planner
        self.a_star = AStar(robot_radius=0.3, res=self.grid_size)

        self.display_map()

    def display_map(self):
        start_pixel = (int((self.start_world[0] - self.origin[0]) / self.grid_size),
                       int((self.start_world[1] - self.origin[1]) / self.grid_size))
        goal_pixel = (int((self.goal_world[0] - self.origin[0]) / self.grid_size),
                       int((self.goal_world[1] - self.origin[1]) / self.grid_size))
        # Kích thước map gốc
        height, width = self.map_array.shape

        # Tính tọa độ thực
        x_min = self.origin[0]
        x_max = self.origin[0] + width * self.resolution

        y_min = self.origin[1]
        y_max = self.origin[1] + height * self.resolution

        # Kích thước map mới
        height, width = self.grid_map.shape

        # Tính tọa độ thực mới
        x_grid_min = self.origin[0]
        x_grid_max = self.origin[0] + width * self.grid_size

        y_grid_min = self.origin[1]
        y_grid_max = self.origin[1] + height * self.grid_size

        # Draw compare
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))

        # ============================================== Original map ==============================================
        axes[0].imshow(
            self.map_array,
            cmap="gray",
            origin="lower",
            extent=[x_min, x_max, y_min, y_max]
        )

        # Vẽ điểm start
        axes[0].scatter(
            self.start_world[0],
            self.start_world[1],
            s=100,
            c='green',
            marker='o',
            label='Start'
        )

        # Vẽ điểm goal
        axes[0].scatter(
            self.goal_world[0],
            self.goal_world[1],
            s=100,
            c='red',
            marker='o',
            label='Goal'
        )

        axes[0].set_title("Original Map")
        axes[0].set_xlabel("X (m)")
        axes[0].set_ylabel("Y (m)")
        axes[0].grid(color='blue', linewidth=0.5)
        axes[0].legend()

        # ============================================== Map 0.3m ==============================================
        axes[1].imshow(
            self.grid_map,
            cmap='gray_r',
            origin='lower',
            extent=[x_grid_min, x_grid_max, y_grid_min, y_grid_max]
        )

        # Vẽ điểm start
        axes[1].scatter(
            start_pixel[0],
            start_pixel[1],
            s=100,
            c='green',
            marker='o',
            label='Start'
        )

        # Vẽ điểm goal
        axes[1].scatter(
            goal_pixel[0],
            goal_pixel[1],
            s=100,
            c='red',
            marker='o',
            label='Goal'
        )

        axes[1].set_title("0.3m Grid Map")
        axes[1].set_xlabel("X (m)")
        axes[1].set_ylabel("Y (m)")
        axes[1].grid(color='blue', linewidth=0.5)
        axes[1].legend()

        # VẼ LƯỚI Ô

        # Tick theo kích thước cell
        x_ticks = np.arange(x_grid_min, x_grid_max + self.grid_size, self.grid_size)
        y_ticks = np.arange(y_grid_min, y_grid_max + self.grid_size, self.grid_size)

        axes[1].set_xticks(x_ticks)
        axes[1].set_yticks(y_ticks)

        # Vẽ grid
        axes[1].grid(
            which='both',
            color='blue',
            linewidth=0.5
        )

        # Giữ tỉ lệ vuông
        axes[1].set_aspect('equal')

        plt.tight_layout()
        plt.show()


# Entry point
def main():
    rclpy.init()
    planner = Planner()
    rclpy.spin(planner)


if __name__ == "__main__":
    main()