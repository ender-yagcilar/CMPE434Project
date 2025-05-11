import numpy as np

class PurePursuitController:
    def __init__(self, lookahead_distance=1.5, wheelbase=0.33):
        """
        Initializes the Pure Pursuit controller.
        
        :param lookahead_distance: Distance to the lookahead point (L).
        :param wheelbase: Distance between the front and rear wheels (L_f).
        """
        self.lookahead_distance = lookahead_distance
        self.wheelbase = wheelbase  # Needed for computing steering angle
        
    
    def find_lookahead_point(self, position, waypoints, current_idx):
        """
        Finds the lookahead point from the given waypoints.

        :param position: (x, y) current position of the car.
        :param waypoints: List of (x, y) waypoints.
        :param current_idx: Index of the current waypoint being followed.
        :return: (x, y) lookahead point, new waypoint index.
        """
        for i in range(current_idx, len(waypoints)):  # Start from current waypoint
            waypoint = waypoints[i]
            distance = np.linalg.norm(np.array(waypoint) - np.array(position))

            if distance >= self.lookahead_distance:
                return waypoint, i  # Return the new waypoint index as well

        return waypoints[-1], len(waypoints) - 1  # Return last waypoint if none found


    def compute_steering_angle(self, car_position, car_yaw, waypoints, current_idx):
        lookahead_point, new_idx = self.find_lookahead_point(car_position, waypoints, current_idx)
    
        # Compute steering angle as usual
        dx = lookahead_point[0] - car_position[0]
        dy = lookahead_point[1] - car_position[1]
        target_angle = np.arctan2(dy, dx)
        angle_diff = target_angle - np.radians(car_yaw)
        
        return np.arctan2(2.0 * self.lookahead_distance * np.sin(angle_diff), self.lookahead_distance), new_idx

