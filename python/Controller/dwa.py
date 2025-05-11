import numpy as np

class DWA:
    def __init__(self,
                 max_speed=10,
                 min_speed=-10,
                 max_yawrate=np.radians(40.0),
                 max_accel=2,
                 max_dyawrate=np.radians(40.0),
                 dt=10000,
                 predict_time=3.0,
                 robot_radius=0.5,
                 to_goal_cost_gain=1.0,
                 speed_cost_gain=1.0,
                 obstacle_cost_gain=1.0):

        self.MAX_SPEED = max_speed
        self.MIN_SPEED = min_speed
        self.MAX_YAWRATE = max_yawrate
        self.MAX_ACCEL = max_accel
        self.MAX_DYAWRATE = max_dyawrate
        self.DT = dt
        self.PREDICT_TIME = predict_time
        self.ROBOT_RADIUS = robot_radius
        self.TO_GOAL_COST_GAIN = to_goal_cost_gain
        self.SPEED_COST_GAIN = speed_cost_gain
        self.OBSTACLE_COST_GAIN = obstacle_cost_gain

    def motion(self, state, control):
        x, y, yaw, v, omega = state
        v += control[0] * self.DT
        omega += control[1] * self.DT
        x += v * np.cos(yaw) * self.DT
        y += v * np.sin(yaw) * self.DT
        yaw += omega * self.DT
        return [x, y, yaw, v, omega]

    def calc_dynamic_window(self, state):
        Vs = [self.MIN_SPEED, self.MAX_SPEED, -self.MAX_YAWRATE, self.MAX_YAWRATE]
        Vd = [
            state[3] - self.MAX_ACCEL * self.DT,
            state[3] + self.MAX_ACCEL * self.DT,
            state[4] - self.MAX_DYAWRATE * self.DT,
            state[4] + self.MAX_DYAWRATE * self.DT
        ]
        return [
            max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])
        ]

    def calc_trajectory(self, state, v, omega):
        trajectory = [state.copy()]
        for _ in np.arange(0, self.PREDICT_TIME, self.DT):
            state = self.motion(state, [v, omega])
            state[3] = v
            state[4] = omega
            trajectory.append(state.copy())
        return trajectory

    def calc_to_goal_cost(self, trajectory, goal):
        dx = goal[0] - trajectory[-1][0]
        dy = goal[1] - trajectory[-1][1]
        error_angle = np.arctan2(dy, dx)
        cost = abs(error_angle - trajectory[-1][2])
        return self.TO_GOAL_COST_GAIN * cost

    def calc_obstacle_cost(self, trajectory, obstacles):
        cost = 0.0
        for step in trajectory:
            for ox, oy in obstacles:
                d = np.hypot(ox - step[0], oy - step[1])
                if d <= self.ROBOT_RADIUS:
                    return float("Inf")  # collision
                cost += 1.0 / d
        return self.OBSTACLE_COST_GAIN * cost

    def plan(self, state, goal, obstacles):
        dw = self.calc_dynamic_window(state)
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = [state]

        for v in np.arange(dw[0], dw[1], 0.1):
            for omega in np.arange(dw[2], dw[3], np.radians(5.0)):
                trajectory = self.calc_trajectory(state.copy(), v, omega)
                to_goal_cost = self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.SPEED_COST_GAIN * (self.MAX_SPEED - trajectory[-1][3])
                obstacle_cost = self.calc_obstacle_cost(trajectory, obstacles)
                total_cost = to_goal_cost + speed_cost + obstacle_cost

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_u = [v, omega]
                    best_trajectory = trajectory

        return best_u, best_trajectory
