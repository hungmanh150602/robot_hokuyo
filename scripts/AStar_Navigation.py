#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import rclpy

from rclpy.node import Node
from load_map import load_map
from a_star import AStar
from path_follower import LineSegmentPathFollower


class Planner(Node):
    def __init__(self):
        super().__init__('Planner')
        # Load map + robot state
        self.robot_radius = 0.3
        self.grid_size = 0.3

        self.raw_map, self.astar_map, self.resolution, self.origin = load_map(
            # "maps/map.yaml",
            "/home/hungubuntu/ros2_workspace/src/robot_hokuyo/maps/my_map_save.yaml",
            self.grid_size
        )

        # Start and Goal (in world coordinates)
        self.start_world = (0.0, 0.0)
        self.goal_world = (3.8, 1.5)

        # Convert world to pixel
        self.grid_h, self.grid_w = self.astar_map.shape
        if 0: # flipud
            self.start_pixel = (int((self.start_world[0] - self.origin[0]) / self.grid_size),
                                int((self.start_world[1] - self.origin[1]) / self.grid_size))
            self.goal_pixel = (int((self.goal_world[0] - self.origin[0]) / self.grid_size),
                               int((self.goal_world[1] - self.origin[1]) / self.grid_size))
        else:
            self.start_pixel = (int((self.start_world[0] - self.origin[0]) / self.grid_size),
                                self.grid_h - int((self.start_world[1] - self.origin[1]) / self.grid_size) - 1)
            self.goal_pixel = (int((self.goal_world[0] - self.origin[0]) / self.grid_size),
                               self.grid_h - int((self.goal_world[1] - self.origin[1]) / self.grid_size) - 1)

        # A* path planner
        self.a_star = AStar(robot_radius=self.robot_radius, res=self.grid_size)

        self.raw_path, self.pruned_path, self.smooth_path, _ = self.a_star.find_path(self.astar_map, self.start_pixel,
                                                                                     self.goal_pixel)

        self.path_world = []

        for px, py in self.smooth_path:
            wx = self.origin[0] + px * self.grid_size
            wy = self.origin[1] + (self.grid_h - py - 1) * self.grid_size

            self.path_world.append([wx, wy])

        self.path_world = np.array(self.path_world).T

        robot_state = np.array([
            self.path_world[0, 0],  # x
            self.path_world[1, 0],  # y
            0.0  # theta
        ])

        follower = LineSegmentPathFollower(
            path_xy=self.path_world,
            robot_state=robot_state,
            Ts=0.05,
            goal_tol=0.5,
            kp=2.8,
            kd=0.5,
            v_nominal=0.05
        )

        self.X, self.Y, self.TH, self.V, self.W = follower.run()

        self.display_map(self.raw_map, self.astar_map, self.raw_path, self.pruned_path, self.smooth_path)

    def display_map(self, raw_map, astar_grid, raw_path, pruned_path, smooth_path):
        # SIZE
        h1, w1 = raw_map.shape

        # Tính tọa độ thực
        x_min = self.origin[0]
        x_max = self.origin[0] + w1 * self.resolution

        y_min = self.origin[1]
        y_max = self.origin[1] + h1 * self.resolution

        # FIGURE
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        # ============================== ORIGINAL MAP ==============================
        axes[0].imshow(
            raw_map,
            cmap='gray',
            origin='upper',
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

        # Vẽ robot trajectory
        axes[0].plot(
            self.X,
            self.Y,
            'b-',
            linewidth=2,
            label='Robot Trajectory'
        )

        # Vẽ reference path
        axes[0].plot(
            self.path_world[0],
            self.path_world[1],
            'r--',
            linewidth=2,
            label='Reference Path'
        )

        axes[0].set_title("Original Map")
        axes[0].grid(color='blue', linewidth=0.5)
        axes[0].legend()

        # ============================== GRID MAP 0.3m ==============================
        axes[1].imshow(
            astar_grid,
            cmap='gray_r',
            origin='upper'
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

        axes[1].set_title("A* Grid Map (0.3m)")

        # Grid line
        axes[1].set_xticks(np.arange(-0.5, self.grid_w, 1))
        axes[1].set_yticks(np.arange(-0.5, self.grid_h, 1))

        axes[1].grid(
            color='blue',
            linewidth=0.5
        )

        # Vẽ raw path
        if raw_path is not None:
            path_x = [p[0] for p in raw_path]
            path_y = [p[1] for p in raw_path]

            axes[1].plot(
                path_x,
                path_y,
                color='yellow',
                linewidth=2,
                label='Raw Path'
            )

        # Vẽ pruned_path path
        if pruned_path is not None:
            path_x = [p[0] for p in pruned_path]
            path_y = [p[1] for p in pruned_path]

            axes[1].plot(
                path_x,
                path_y,
                color='blue',
                linewidth=2,
                label='Pruned Path'
            )

        # Vẽ smooth_path path
        if smooth_path is not None:
            path_x = [p[0] for p in smooth_path]
            path_y = [p[1] for p in smooth_path]

            axes[1].plot(
                path_x,
                path_y,
                color='brown',
                linewidth=2,
                label='Smooth Path'
            )

        axes[1].set_aspect('equal')
        axes[1].legend()

        plt.tight_layout()
        plt.show()


# Entry point
def main():
    rclpy.init()
    planner = Planner()
    rclpy.spin(planner)


if __name__ == "__main__":
    main()
