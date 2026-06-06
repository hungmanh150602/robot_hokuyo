import numpy as np


class LineSegmentPathFollower:
    def __init__(self, path_xy, robot_state, Ts=0.1, goal_tol=0.08, kp=5.0, ki=0.0, kd=0.3, wmax=8.0, v_nominal=0.4):
        """
        path_xy : np.array shape (2, N)  -> [[x0,x1,...],[y0,y1,...]]
        """
        self.T = path_xy
        self.N = path_xy.shape[1]
        self.Ts = Ts
        self.goal_tol = goal_tol

        # PID
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wmax = wmax
        self.v_nominal = v_nominal

        # trạng thái robot
        self.q = robot_state

        # PID memory
        self.I = 0.0
        self.prev_e = 0.0

        # index đoạn thẳng
        self.i = 0

        # log
        self.X, self.Y, self.TH = [], [], []
        self.Vcmd, self.Wcmd = [], []

    # ============================
    @staticmethod
    def wrap_to_pi(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    # ============================
    def reached_goal(self):
        goal = self.T[:, -1]
        return np.linalg.norm(self.q[:2] - goal) < self.goal_tol

    # ============================
    def run_step(self):
        """
        Thực hiện 1 bước điều khiển Ts
        Trả về False nếu đã kết thúc
        """
        if self.reached_goal() or self.i >= self.N - 1:
            return False

        A = self.T[:, self.i]
        C = self.T[:, self.i + 1]
        v = C - A

        # bỏ đoạn rỗng
        if v @ v < 1e-6:
            self.i += 1
            return True

        vN = np.array([v[1], -v[0]])
        r = self.q[:2] - A

        # chiếu lên đoạn
        u = (v @ r) / v @ v
        if u > 1.0 and self.i < self.N - 2:
            self.i += 1
            return True

        # sai lệch vuông góc
        vN_norm2 = vN @ vN
        if vN_norm2 > 1e-6:
            dn = (vN @ r) / vN_norm2
        else:
            dn = 0.0

        # góc tham chiếu
        phi_line = np.arctan2(v[1], v[0])
        phi_corr = np.arctan(4 * dn)
        phi_ref = self.wrap_to_pi(phi_line + phi_corr)

        # PID góc
        e = self.wrap_to_pi(phi_ref - self.q[2])
        self.I += e * self.Ts
        D = (e - self.prev_e) / self.Ts
        self.prev_e = e

        w = self.kp * e + self.ki * self.I + self.kd * D
        w = np.clip(w, -self.wmax, self.wmax)

        # giảm tốc khi gần goal
        dist = np.linalg.norm(self.q[:2] - self.T[:, -1])
        vcmd = self.v_nominal * np.cos(e) * min(1.0, dist / 1.0)

        # cập nhật robot
        self.q[0] += self.Ts * vcmd * np.cos(self.q[2])
        self.q[1] += self.Ts * vcmd * np.sin(self.q[2])
        self.q[2] = self.wrap_to_pi(self.q[2] + self.Ts * w)

        # log
        self.X.append(self.q[0])
        self.Y.append(self.q[1])
        self.TH.append(self.q[2])
        self.Vcmd.append(vcmd)
        self.Wcmd.append(w)

        return True

    # ============================
    def run(self, max_steps=10000):
        """
        Chạy cho tới khi tới goal hoặc đạt max_steps
        """
        for _ in range(max_steps):
            if not self.run_step():
                break

        return (
            np.array(self.X),
            np.array(self.Y),
            np.array(self.TH),
            np.array(self.Vcmd),
            np.array(self.Wcmd)
        )
