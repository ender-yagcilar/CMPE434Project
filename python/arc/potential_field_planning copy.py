import numpy as np
import matplotlib.pyplot as plt
from collections import deque


class PotentialFieldPlanner:
    def __init__(self, grid_size=0.5, robot_radius=5.0, kp=5.0, eta=100.0, area_width=30.0, show_animation=True):
        self.grid_size = grid_size
        self.robot_radius = robot_radius
        self.KP = kp
        self.ETA = eta
        self.AREA_WIDTH = area_width
        self.SHOW_ANIMATION = show_animation
        self.OSCILLATIONS_DETECTION_LENGTH = 3

    def plan(self, start, goal, obstacles):
        sx, sy = start
        gx, gy = goal
        ox, oy = zip(*obstacles)

        pmap, minx, miny = self.calc_potential_field(gx, gy, ox, oy, sx, sy)
        print(pmap)

        d = np.hypot(sx - gx, sy - gy)
        ix = round((sx - minx) / self.grid_size)
        iy = round((sy - miny) / self.grid_size)
        gix = round((gx - minx) / self.grid_size)
        giy = round((gy - miny) / self.grid_size)

        if self.SHOW_ANIMATION:
            self.draw_heatmap(pmap)
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")

        rx, ry = [sx], [sy]
        motion = self.get_motion_model()
        previous_ids = deque()

        while d >= self.grid_size:
            minp = float("inf")
            minix, miniy = -1, -1
            for mx, my in motion:
                inx = ix + mx
                iny = iy + my
                if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")  # outside area
                else:
                    p = pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix, iy = minix, miniy
            xp = ix * self.grid_size + minx
            yp = iy * self.grid_size + miny
            d = np.hypot(gx - xp, gy - yp)
            rx.append(xp)
            ry.append(yp)

            if self.oscillations_detection(previous_ids, ix, iy):
                print(f"Oscillation detected at ({ix},{iy})!")
                break

            if self.SHOW_ANIMATION:
                plt.plot(ix, iy, ".r")
                plt.pause(0.01)

        print("Goal!!")
        if self.SHOW_ANIMATION:
            plt.show()
        return rx, ry

    def calc_potential_field(self, gx, gy, ox, oy, sx, sy):
        minx = min(min(ox), sx, gx) - self.AREA_WIDTH / 2.0
        miny = min(min(oy), sy, gy) - self.AREA_WIDTH / 2.0
        maxx = max(max(ox), sx, gx) + self.AREA_WIDTH / 2.0
        maxy = max(max(oy), sy, gy) + self.AREA_WIDTH / 2.0
        xw = int(round((maxx - minx) / self.grid_size))
        yw = int(round((maxy - miny) / self.grid_size))

        pmap = [[0.0 for _ in range(yw)] for _ in range(xw)]

        for ix in range(xw):
            x = ix * self.grid_size + minx
            for iy in range(yw):
                y = iy * self.grid_size + miny
                ug = 0.5 * self.KP * np.hypot(x - gx, y - gy)
                uo = self.calc_repulsive_potential(x, y, ox, oy)
                pmap[ix][iy] = ug + uo

        return pmap, minx, miny

    def calc_repulsive_potential(self, x, y, ox, oy):
        min_dist = float("inf")
        for obx, oby in zip(ox, oy):
            dist = np.hypot(x - obx, y - oby)
            if dist < min_dist:
                min_dist = dist

        dq = min_dist
        if dq <= self.robot_radius:
            dq = max(dq, 0.1)
            return 0.5 * self.ETA * (1.0 / dq - 1.0 / self.robot_radius) ** 2
        return 0.0

    def get_motion_model(self):
        return [[1, 0], [0, 1], [-1, 0], [0, -1],
                [-1, -1], [-1, 1], [1, -1], [1, 1]]

    def oscillations_detection(self, previous_ids, ix, iy):
        previous_ids.append((ix, iy))
        if len(previous_ids) > self.OSCILLATIONS_DETECTION_LENGTH:
            previous_ids.popleft()

        return len(set(previous_ids)) < len(previous_ids)

    def draw_heatmap(self, data):
        data = np.array(data).T
        plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)
        plt.axis("equal")
        plt.grid(True)


# Example usage
if __name__ == "__main__":
    planner = PotentialFieldPlanner()

    start = (0.0, 10.0)
    goal = (30.0, 30.0)
    obstacles = [(15.0, 25.0), (5.0, 15.0), (20.0, 26.0), (25.0, 25.0)]

    rx, ry = planner.plan(start, goal, obstacles)

    if planner.SHOW_ANIMATION:
        plt.plot(rx, ry, "-r")
        plt.show()
