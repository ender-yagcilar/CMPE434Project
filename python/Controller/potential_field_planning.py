import numpy as np

class PotentialFieldPlanner:
    def __init__(self,
                 k_att=1.0,
                 wheelbase=2.0,
                 max_velocity=10.0,
                 max_steering_deg=5.0):
        self.k_att = k_att
        #self.k_rep = k_rep
        #self.repulsion_radius = repulsion_radius
        self.wheelbase = wheelbase
        self.max_velocity = max_velocity
        self.max_steering_deg = max_steering_deg

        self.alpha = 0.2# smoothing factor between 0 and 1
        self.prev_velocity = 0.0
        self.prev_steering = 0.0

        self.obstacles = []  # List of dictionaries: {"pos": np.array, "k_rep": float, "radius": float}


    def smooth(self, current, previous):
        return self.alpha * current + (1 - self.alpha) * previous
    
    def add_obstacle(self, x, y, k_rep, radius):
        self.obstacles.append({
            "pos": np.array([x, y]),
            "k_rep": k_rep,
            "radius": radius
        })
    def flush_obstacles(self):
        """Remove all obstacles from the planner."""
        self.obstacles.clear()

    def print_obstacle_count(self):
        print(f"Number of obstacles: {len(self.obstacles)}")


    def attractive_force(self, pos, goal):
        return self.k_att * (goal - pos)

    def repulsive_force(self, pos):
        F_rep = np.zeros(2)
        for obs in self.obstacles:
            obs_pos = obs["pos"]
            k_rep = obs["k_rep"]
            radius = obs["radius"]

            direction = pos - obs_pos
            distance = np.linalg.norm(direction)
            if 1e-2 < distance < radius:
                #print(distance)
                repulsion = k_rep * (1.0 / distance - 1.0 / radius)
                repulsion *= 1.0 / (distance ** 2)
                F_rep += repulsion * (direction / distance)
        return F_rep

    def compute_command(self, pos, yaw, goal):
        pos = np.array(pos)
        goal = np.array(goal)

        F_att = self.attractive_force(pos, goal)
        F_rep = self.repulsive_force(pos)
        F_total = F_att + F_rep

        #print("F Total:",F_total, F_att, "+",F_rep,end="-----****------")

        if np.linalg.norm(F_total) < 1e-3:
            return 0.0, 0.0  # stop if almost no force

        target_angle = np.arctan2(F_total[1], F_total[0])
        yaw_error = np.arctan2(np.sin(target_angle - yaw), np.cos(target_angle - yaw))
        
        
        F_total_norm = np.clip(np.linalg.norm(F_total),-6,6)
        # Steering calculation
        curvature = 2 * np.sin(yaw_error) / F_total_norm
        steering_rad = np.arctan(self.wheelbase * curvature)
        steering_deg = yaw_error/np.pi *self.max_steering_deg

        #print("target_angle:",target_angle,"yaw",yaw,"error yaw :",yaw_error)

        # Velocity reduction on sharp turns
        velocity = self.max_velocity * (1.0 - 0.9 * abs(steering_deg) / self.max_steering_deg)

        # Apply smoothing
        #steering_deg = self.smooth(steering_deg, self.prev_steering)

        self.prev_velocity = velocity
        self.prev_steering = steering_deg

        return velocity, steering_deg