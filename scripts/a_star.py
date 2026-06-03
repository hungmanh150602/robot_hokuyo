import heapq
import numpy as np
import math


class AStar:
    def __init__(self, robot_radius=1.0, res=0.5):
        self.robot_radius = robot_radius   # mét
        self.res = res                     # mét/pixel

    def is_collision(self, grid, x, y, robot_radius, res):
        """
        Kiểm tra robot đặt tại pixel (x, y) có chạm vật cản không,
        dựa trên bán kính robot radius (m).
        """
        r_pix = int(robot_radius / res)
        h, w = grid.shape

        for dx in range(-r_pix, r_pix + 1):
            for dy in range(-r_pix, r_pix + 1):
                if dx * dx + dy * dy <= r_pix * r_pix:
                    nx = x + dx
                    ny = y + dy

                    # ra ngoài map => va chạm
                    if not (0 <= nx < h and 0 <= ny < w):
                        return True

                    # 0 = obstacle
                    if grid[nx][ny] == 1:
                        return True

        return False

    # ============================================
    # A* Search (4 directions or 8 directions)
    # ============================================
    def astar(self, grid, start, goal):
        h, w = grid.shape

        g = np.full((h, w), np.inf)
        g[start] = 0

        closed = np.zeros((h, w), dtype=bool)
        parent = {}

        pq = [(0, start)]

        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while pq:
            _, (x, y) = heapq.heappop(pq)

            if closed[x, y]:
                continue
            closed[x, y] = True

            if (x, y) == goal:
                return self.reconstruct_path(parent, goal)

            for dx, dy in dirs:
                nx, ny = x + dx, y + dy

                if not (0 <= nx < h and 0 <= ny < w):
                    continue
                if grid[nx, ny]:
                    continue
                if closed[nx, ny]:
                    continue
                # if self.is_collision(grid, nx, ny, self.robot_radius, self.res):
                #     continue

                cost = g[x, y] + math.hypot(dx, dy)
                if cost < g[nx, ny]:
                    g[nx, ny] = cost
                    parent[(nx, ny)] = (x, y)
                    f = cost + self.euclid((nx, ny), goal)
                    heapq.heappush(pq, (f, (nx, ny)))

        return None

    # ============================================
    # Heuristic
    # ============================================
    @staticmethod
    def euclid(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    # ============================================
    # Reconstruct A* path
    # ============================================
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))

    # ============================================
    # Loại bỏ các điểm dư thừa trong path A* bằng phương pháp 3 điểm
    # ============================================
    def is_line_collision_free(self, grid, p1, p2, step=0.5):
        """
        Kiểm tra đoạn thẳng p1 → p2 có va chạm không
        step: khoảng cách mẫu (m)
        """
        p1 = np.array(p1)
        p2 = np.array(p2)

        dist = np.linalg.norm(p2 - p1)
        n = int(dist / step)

        for i in range(n + 1):
            t = i / max(n, 1)
            p = p1 + t * (p2 - p1)

            x = int(round(p[0]))
            y = int(round(p[1]))

            # if self.is_collision(grid, x, y, self.robot_radius, self.res):
            #     return False

        return True

    def prune_path_3points_collision(self, grid, path):
        """
        Loại bỏ điểm giữa nếu đoạn nối 2 đầu không va chạm
        """
        if len(path) < 3:
            return path

        pruned = [path[0]]
        i = 1

        while i < len(path) - 1:
            A = pruned[-1]
            B = path[i]
            C = path[i + 1]

            # Nếu tạo tam giác nhưng AC không va chạm → bỏ B
            if self.is_line_collision_free(grid, A, C, step=self.res / 2):
                i += 1
                continue

            # Nếu không bỏ được → giữ B
            pruned.append(B)
            i += 1

        pruned.append(path[-1])
        return pruned

    def prune_path_until_converged(self, grid, path, max_iter=1):
        """
        Lặp prune 3-point + LOS cho đến khi path không còn thay đổi
        """
        prev_len = len(path)
        current = path

        for _ in range(max_iter):
            current = self.prune_path_3points_collision(grid, current)

            if len(current) == prev_len:
                break  # hội tụ

            prev_len = len(current)

        return current

    # ============================================
    # Bézier using matrix form T * M * G
    # ============================================
    def quadratic_bezier(self, P0, P1, P2, n=10):
        # B(t) = (1−t)^2.P0 + 2(1−t)t.P1 + t^2.P2
        t = np.linspace(0, 1, n)

        T = np.column_stack((t ** 2, t, np.ones_like(t)))  # (n, 3)

        M = np.array([
            [1, -2, 1],
            [-2, 2, 0],
            [1, 0, 0]
        ])

        P = np.vstack((P0, P1, P2))

        return T @ M @ P


    def is_corner(self, p0, p1, p2, angle_thresh=10):
        v1 = (p0 - p1).astype(np.float64)
        v2 = (p2 - p1).astype(np.float64)

        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)

        if n1 < 1e-6 or n2 < 1e-6:
            return False

        v1 /= n1
        v2 /= n2

        angle = np.degrees(
            np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
        )

        return angle > angle_thresh

    # ============================================
    # Smooth A* path using overlapping cubic Bézier
    # ============================================
    def smooth_path(self, path,
                    n_points=30,
                    angle_thresh=10):
        path = np.array(path)
        N = len(path)

        if N <= 2:
            return path, None

        smooth = [path[0]]

        for i in range(1, N - 1):
            p_prev = path[i - 1]
            p_curr = path[i]
            p_next = path[i + 1]

            if not self.is_corner(p_prev, p_curr, p_next, angle_thresh):
                smooth.append(p_curr)
                continue

            # =========================
            # Bézier bậc 2 bằng midpoint
            # =========================
            P0 = 0.5 * (p_prev + p_curr)
            P1 = p_curr
            P2 = 0.5 * (p_curr + p_next)

            curve = self.quadratic_bezier(P0, P1, P2, n_points)

            smooth.extend(curve)

        smooth.append(path[-1])

        return np.array(smooth)

    # ============================================
    # Main API: A* + Optional smoothing
    # ============================================
    def find_path(self, grid, start, goal, smooth=True):
        raw = self.astar(grid, start, goal)

        if raw is None:
            print("Không tìm thấy đường đi!")
            return None, None, None

        # loại điểm dư
        pruned = self.prune_path_until_converged(grid, raw)

        if smooth:
            smooth_path = self.smooth_path(pruned, 10)
            return raw, pruned, smooth_path

        return raw, pruned, None
