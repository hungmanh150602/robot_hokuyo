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
        self.grid_size = 0.5

        self.map_array, self.grid_map, self.resolution, self.origin = load_map(
            "/home/hungubuntu/ros2_workspace/src/my_first_pkg/maps/map.yaml",
            # "/home/hungubuntu/ros2_workspace/src/robot_hokuyo/maps/my_map_save.yaml",
            self.grid_size
        )

        # Start and Goal (in world coordinates)
        self.start_world = (0.0, 0.0)
        self.goal_world = (1.0, 4.0)

        # Convert world to pixel
        self.start_pixel = (int((self.start_world[0] - self.origin[0]) / self.grid_size),
                            int((self.start_world[1] - self.origin[1]) / self.grid_size))
        self.goal_pixel = (int((self.goal_world[0] - self.origin[0]) / self.grid_size),
                            int((self.goal_world[1] - self.origin[1]) / self.grid_size))

        # A* path planner
        self.a_star = AStar(robot_radius=0.5, res=self.grid_size)

        self.raw_path, self.pruned_path, self.smooth_path = self.a_star.find_path(self.grid_map, self.start_pixel, self.goal_pixel)

        # self.get_logger().info(self.raw_path)

        self.display_map()

    def display_map(self):
        # Kích thước map gốc
        height, width = self.map_array.shape

        # Tính tọa độ thực
        x_min = self.origin[0]
        x_max = self.origin[0] + width * self.resolution

        y_min = self.origin[1]
        y_max = self.origin[1] + height * self.resolution

        # Draw compare
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))

        # ============================================== Original map ==============================================
        axes[0].imshow(
            self.map_array,
            cmap="gray",
            # origin="upper",
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
            # origin='lower',
        )

        # Vẽ điểm start
        axes[1].scatter(
            self.start_pixel[0],
            self.start_pixel[1],
            s=100,
            c='green',
            marker='o',
            label='Start'
        )

        # Vẽ điểm goal
        axes[1].scatter(
            self.goal_pixel[0],
            self.goal_pixel[1],
            s=100,
            c='red',
            marker='o',
            label='Goal'
        )

        # Vẽ raw path
        if self.raw_path is not None:
            # Kích thước map mới
            height, width = self.grid_map.shape

            path_x = [p[0] + 0.5 for p in self.raw_path]
            path_y = [p[1] + 0.5 for p in self.raw_path]

            axes[1].plot(
                path_x,
                path_y,
                color='yellow',
                linewidth=2,
                label='Raw Path'
            )

        axes[1].set_title("0.3m Grid Map")
        axes[1].set_xlabel("X")
        axes[1].set_ylabel("Y")
        axes[1].legend()

        # VẼ LƯỚI Ô
        axes[1].set_xticks(np.arange(-0.5, width, 1))
        axes[1].set_yticks(np.arange(-0.5, height, 1))

        # Vẽ grid
        axes[1].grid(color='blue', linewidth=0.5)

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