#SON NOKTAYA DÜŞÜK THRESHOLDLA GİTSİN
import time
import mujoco
import mujoco.viewer
import random
import numpy as np
import matplotlib.pyplot as plt
import math
import scipy as sp

import cmpe434_dungeon as dungeon

import Controller.KeyboardControl as KeyboardControl
from Controller.probabilistic_road_map import PRM 
from Controller.PurePursuitController import PurePursuitController
from Controller.potential_field_planning import PotentialFieldPlanner
from Controller.dwa import DWA
# Helper construsts for the viewer for pause/unpause functionality.
paused = False


class RacingCarControl:
    def __init__(self, initial_position, initial_orientation,Goal_Pos, LocalController):

        self.InitialPosition = initial_position
        self.InitialOrientation = initial_orientation
        self.FinalGoal = Goal_Pos
        self.LocalController = LocalController
        self.LocalControllerNumber = LocalController
        self.robot_size = 0.5

        self.waypoints = []
        self.current_idx = 0

        self.status = "initialized" 
        self.stuck_start_time = None
        self.in_reverse_mode = False
        self.reverse_start_time = None
        self.reverse_duration = 3.0  # seconds
        self.stuck_threshold = 5.0   # seconds


        self.ControlVelocity = 0
        self.ControlAttitude = 0
        
        self.waypoint_threshold = 1# Minimum distance to switch waypoints
        if self.LocalControllerNumber == 1:
            self.LocalController = KeyboardControl.KeyboardControl()

        elif self.LocalControllerNumber == 5:
            self.LocalController = PurePursuitController(lookahead_distance=1)

        elif self.LocalControllerNumber == 6:
            self.LocalController = PotentialFieldPlanner(k_att=10,wheelbase=0.33,max_velocity=5.0,max_steering_deg=10.0)
            
        #DWA
        elif self.LocalControllerNumber == 7:
            self.LocalController = DWA()
            # Auto-tuned parameters for better performance
            self.LocalController.MAX_SPEED = 10
            self.LocalController.MIN_SPEED = 0.0
            self.LocalController.MAX_YAWRATE = np.radians(90)
            self.LocalController.MAX_DYAWRATE = np.radians(180)
            self.LocalController.MAX_ACCEL = 10
            self.LocalController.MAX_DYAWACCEL = np.radians(180)
            self.LocalController.V_RESOLUTION = 0.02
            self.LocalController.OMEGA_RESOLUTION = np.radians(1)
            self.LocalController.PREDICT_TIME = 10

            # Cost gains
            self.LocalController.TO_GOAL_COST_GAIN = 1.2
            self.LocalController.SPEED_COST_GAIN = 0.1
            self.LocalController.OBSTACLE_COST_GAIN = 0.4
            
        

    def plan_globally(self,GlobalController=1,Walls=0):
        self.GlobalPlannerType = GlobalController

        if self.GlobalPlannerType == 1:
            is_path_found = False

            while not is_path_found:
                self.GlobalPlanner = PRM(self.InitialPosition[0], self.InitialPosition[1], self.FinalGoal[0], self.FinalGoal[1],
                              Walls[0] , Walls[1], self.robot_size, rng=None)

                rx, ry,is_path_found = PRM.prm_planning(self.GlobalPlanner,self.InitialPosition[0], self.InitialPosition[1], self.FinalGoal[0], self.FinalGoal[1],
                                      Walls[0] , Walls[1], self.robot_size, rng=None)

            i = len(rx)-1
            while i != 0:
                self.waypoints.append([rx[i] ,ry[i]])
                i-=1
            self.waypoints.append(self.FinalGoal)
    
    def plan_locally(self, d, CurrentPosition, CurrentOrientation, static_obstacles, dynamic_obstacles):
        self.initialize_recovery_state()
        self.update_waypoint(CurrentPosition)
        current_goal = self.get_current_goal()

        if self.check_goal_reached(CurrentPosition):
            self.ControlVelocity = 0
            self.ControlAttitude = 0
            return

        current_time = time.time()

        if self.handle_recovery_behavior(current_time):
            return

        if self.check_and_trigger_recovery(current_time):
            return

        if self.LocalControllerNumber == 1:
            self.run_keyboard_control(CurrentPosition, CurrentOrientation, static_obstacles, dynamic_obstacles, current_goal)
        elif self.LocalControllerNumber == 5:
            self.run_pure_pursuit_control(CurrentPosition, CurrentOrientation)
        elif self.LocalControllerNumber == 6:
            self.run_potential_field_control(CurrentPosition, CurrentOrientation, static_obstacles, dynamic_obstacles, current_goal)
        elif self.LocalControllerNumber == 7:
            self.run_dwa_control(d,CurrentPosition, CurrentOrientation, static_obstacles, dynamic_obstacles, current_goal)

        self.update_status(d, CurrentPosition, CurrentOrientation)

    def initialize_recovery_state(self):
        if not hasattr(self, 'stuck_time'):
            self.stuck_time = None
            self.recovery_mode = False
            self.recovery_start_time = None
            self.recovery_duration = 3
            self.recovery_velocity = -4
            self.recovery_steering = 0
            self.previous_controller_number = None

    def get_current_goal(self):
        return self.waypoints[self.current_idx] if self.current_idx < len(self.waypoints) else self.waypoints[-1]

    def check_goal_reached(self, CurrentPosition):
        return self.current_idx >= len(self.waypoints) or self.is_close_to_point(CurrentPosition, self.waypoints[-1], threshold=0.8)

    def handle_recovery_behavior(self, current_time):
        if self.recovery_mode:
            if current_time - self.recovery_start_time < self.recovery_duration:
                self.ControlVelocity = self.recovery_velocity
                self.ControlAttitude = self.recovery_steering
                print(f"[RECOVERY MODE] Reversing with velocity {self.ControlVelocity}")
            else:
                print("[RECOVERY MODE] Recovery completed. Resuming previous controller.")
                self.recovery_mode = False
                self.LocalControllerNumber = self.previous_controller_number
                self.stuck_time = None
            return True
        return False

    def check_and_trigger_recovery(self, current_time):
        if self.status == "stuck":
            if self.stuck_time is None:
                self.stuck_time = current_time
            elif current_time - self.stuck_time >= 5:
                print("[STUCK DETECTED] Switching to recovery mode.")
                self.recovery_mode = True
                self.recovery_start_time = current_time
                self.previous_controller_number = self.LocalControllerNumber
                self.ControlVelocity = self.recovery_velocity
                self.ControlAttitude = self.recovery_steering
                return True
        else:
            self.stuck_time = None
        return False

    def run_keyboard_control(self, pos, orient, static_obs, dynamic_obs, goal):
        self.ControlVelocity, self.ControlAttitude = self.LocalController.update(pos, orient)
        pf = PotentialFieldPlanner(k_att=3, wheelbase=0.33, max_velocity=3.0, max_steering_deg=10.0)
        pf.flush_obstacles()
        for obs in static_obs[4::5]:
            pf.add_obstacle(obs[0], obs[1], k_rep=30, radius=0.5)
        for obs in dynamic_obs:
            pf.add_obstacle(obs[0], obs[1], k_rep=6, radius=2)
        pf.compute_command(pos, np.deg2rad(self.get_yaw_from_quaternion(orient)), goal)

    def run_pure_pursuit_control(self, pos, orient):
        car_yaw = self.get_yaw_from_quaternion(orient)
        steering_angle, self.current_idx = self.LocalController.compute_steering_angle(
            pos, car_yaw, self.waypoints, self.current_idx
        )
        self.ControlVelocity = 1 if self.current_idx >= len(self.waypoints)-2 else 5
        self.ControlAttitude = steering_angle * 5

    def run_potential_field_control(self, pos, orient, static_obs, dynamic_obs, goal):
        self.LocalController.flush_obstacles()
        for obs in static_obs[4::4]:
            self.LocalController.add_obstacle(obs[0], obs[1], k_rep=50, radius=0.6)
        for obs in dynamic_obs:
            self.LocalController.add_obstacle(obs[0], obs[1], k_rep=20, radius=1.5)
        self.ControlVelocity, self.ControlAttitude = self.LocalController.compute_command(
            pos, np.deg2rad(self.get_yaw_from_quaternion(orient)), goal
        )
        #print(f"Velocity: {self.ControlVelocity:.2f}, Steering: {self.ControlAttitude:.2f} degrees")

    def run_dwa_control(self, d,pos, orient, static_obs, dynamic_obs, goal):

        state = [pos[0], pos[1], np.deg2rad(self.get_yaw_from_quaternion(orient)), np.linalg.norm(d.qvel[0:3]), d.qvel[5]]

        # Combine static and dynamic obstacles (if you want to consider both)
        all_obstacles =  dynamic_obs 

        # Run the DWA planner
        control_input, trajectory = self.LocalController.plan(state, goal, all_obstacles)

        # Update internal state variables (optional)
        self.ControlVelocity = control_input[0]
        self.ControlAttitude = control_input[1]
        #print(control_input)


    def update_waypoint(self,CurrentPosition):
        """Switch to the next waypoint if close enough."""
        if self.current_idx < len(self.waypoints):
            waypoint = self.waypoints[self.current_idx]
            distance = np.linalg.norm(np.array(CurrentPosition) - np.array(waypoint))

            if self.current_idx == len(self.waypoints) - 2:
                threshold = 0.3
            else:
                threshold = self.waypoint_threshold

            if distance < threshold and self.current_idx < len(self.waypoints) - 1:
                self.current_idx += 1  # Move to the next waypoint


    def update_status(self, d,CurrentPosition, CurrentOrientation):
        linear_velocity_norm = np.linalg.norm(d.qvel[0:3])
        
        # Store history of position and orientation
        if not hasattr(self, 'prev_position'):
            self.prev_position = CurrentPosition
            self.prev_orientation = CurrentOrientation
            self.status = "initialized"
            return

        cmd_velocity_magnitude = np.linalg.norm(self.ControlVelocity)

        # Check goal
        if self.current_idx >= len(self.waypoints) or self.is_close_to_point(CurrentPosition, self.waypoints[-1], threshold=0.4):
            self.status = "at_goal"
            self.stuck_start_time = None

        # Check for stuck: has velocity but no significant movement
        elif cmd_velocity_magnitude > 0.1 and linear_velocity_norm < 0.1:
            if self.stuck_start_time is None:
                self.stuck_start_time = time.time()
            self.status = "stuck"

        # Moving normally
        elif linear_velocity_norm > 0.1:
            self.status = "moving"
            self.stuck_start_time = None

        # Otherwise, stopped
        else:
            self.status = "unknown"
            self.stuck_start_time = None

        # Update previous pose for next call
        self.prev_position = CurrentPosition
        self.prev_orientation = CurrentOrientation

    def get_car_position(self):
        """Get the car's current (x, y) position."""
        return self.d.qpos[:2]
    
    def get_yaw_from_quaternion(self, quat):
        """Convert quaternion to yaw angle in degrees."""
        w, x, y, z = quat
        yaw_rad = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return np.degrees(yaw_rad)  
    
    def is_close_to_point(self,CurrentPosition, point, threshold=2):
        distance = np.linalg.norm(np.array(CurrentPosition) - np.array(point))

        return distance <= threshold  
        


#############################################################################################################################

def plot_global_map(Walls,InitialPosition,FinalPosition,waypoints):
    # Plot 
    plt.figure(figsize=(8, 8))  # You can adjust the figure size

    plt.scatter(Walls[0], Walls[1], c='black', marker='s', s=20, label='Walls')
    # Plot final position (scaled)
    plt.scatter(FinalPosition[0]*2, FinalPosition[1]*2, c='red', marker='*', s=200, label='Final Position')

    # Plot car position
    plt.scatter(InitialPosition[0], InitialPosition[1], c='blue', marker='o', s=100, label='Car Position')

        # Plot waypoints
    waypoints = np.array(waypoints)  # Make sure it's a numpy array
    plt.scatter(waypoints[:, 0], waypoints[:, 1], c='blue', marker='.', s=30, label='Waypoints')


    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Map')

    plt.axis('equal')  # Equal aspect ratio (so walls don't look stretched)
    plt.grid(True)
    plt.legend()

    plt.show()


def get_wall_obstacles(wall_positions):
    fine_walls = []

    for wall_position in wall_positions:
        x_center, y_center = wall_position
        edge_length = 2
        n = int(edge_length/0.1)

        c1 = [x_center - edge_length/2,y_center + edge_length/2]
        c2 = [x_center + edge_length/2,y_center + edge_length/2]
        c3 = [x_center + edge_length/2,y_center - edge_length/2]
        c4 = [x_center - edge_length/2,y_center - edge_length/2]

        for i in range(n):
            x = c1[0] + i * 0.1
            y = c1[1]
            fine_walls.append((x, y))

        for i in range(n):
            x = c4[0] + i * 0.1
            y = c4[1]
            fine_walls.append((x, y))

        for i in range(n):
            x = c1[0]
            y = c4[1] + i * 0.1
            fine_walls.append((x, y))

        for i in range(n):
            x = c2[0]
            y = c3[1] + i * 0.1
            fine_walls.append((x, y))
        
    
    x_list = [x for x, y in fine_walls]
    y_list = [y for x, y in fine_walls]

    walls_2xn = [x_list, y_list]
    walls_nx2 = fine_walls
    return walls_nx2,walls_2xn


def add_reference_path(viewer,waypoints):
    for waypoint in waypoints:
        geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        mujoco.mjv_initGeom(
        geom,
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=np.array([0.1, 0.1, 0.1]),  # label_size
        pos=np.array([waypoint[0],waypoint[1],0.1]),  # label position
        mat=np.eye(3).flatten(),  # label orientation
        rgba=np.array([1, 1, 1, 1])  # for wapoint
    )
        viewer.user_scn.ngeom += 1

# Pressing SPACE key toggles the paused state.
def mujoco_viewer_callback(keycode):
    global paused
    if keycode == ord(' '):  # Use ord(' ') for space key comparison
        paused = not paused

def main():

    # Uncomment to start with an empty model
    # scene_spec = mujoco.MjSpec() 

    # Load existing XML models
    scene_spec = mujoco.MjSpec.from_file("scenes/empty_floor.xml")

    tiles, rooms, connections = dungeon.generate(3, 2, 8)

    
    for index, r in enumerate(rooms):
        (xmin, ymin, xmax, ymax) = dungeon.find_room_corners(r)
        scene_spec.worldbody.add_geom(name='R{}'.format(index), type=mujoco.mjtGeom.mjGEOM_PLANE, size=[(xmax-xmin)+1, (ymax-ymin)+1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[(xmin+xmax), (ymin+ymax), 0])

    for pos, tile in tiles.items():
        if tile == "#":
            scene_spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[1, 1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[pos[0]*2, pos[1]*2, 0])

    start_pos = random.choice([key for key in tiles.keys() if tiles[key] == "."])
    final_pos = random.choice([key for key in tiles.keys() if tiles[key] == "." and key != start_pos])

    scene_spec.worldbody.add_site(name='start', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5, 0.5, 0.01], rgba=[0, 0, 1, 1],  pos=[start_pos[0]*2, start_pos[1]*2, 0])
    scene_spec.worldbody.add_site(name='finish', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5, 0.5, 0.01], rgba=[1, 0, 0, 1],  pos=[final_pos[0]*2, final_pos[1]*2, 0])


    robot_spec = mujoco.MjSpec.from_file("models/mushr_car/model.xml")

    # Add robots to the scene:
    # - There must be a frame or site in the scene model to attach the robot to.
    # - A prefix is required if we add multiple robots using the same model.
    scene_spec.attach(robot_spec, frame="world", prefix="robot-")
    scene_spec.body("robot-buddy").pos[0] = start_pos[0] * 2
    scene_spec.body("robot-buddy").pos[1] = start_pos[1] * 2

    # Randomize initial orientation
    yaw = np.random.uniform(-np.pi, np.pi)
    euler = np.array([0.0, 0.0, yaw], dtype=np.float64)
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_euler2Quat(quat, euler, 'xyz')
    scene_spec.body("robot-buddy").quat[:] = quat

    # Add obstacles to the scene
    for i, room in enumerate(rooms):
        obs_pos = random.choice([tile for tile in room if tile != start_pos and tile != final_pos])
        scene_spec.worldbody.add_geom(
            name='Z{}'.format(i), 
            type=mujoco.mjtGeom.mjGEOM_CYLINDER, 
            size=[0.2, 0.05, 0.1], 
            rgba=[0.8, 0.0, 0.1, 1],  
            pos=[obs_pos[0]*2, obs_pos[1]*2, 0.08]
        )

    # Initalize our simulation
    # Roughly, m keeps static (model) information, and d keeps dynamic (state) information. 
    m = scene_spec.compile()
    d = mujoco.MjData(m)

    obstacles = [m.geom(i).id for i in range(m.ngeom) if m.geom(i).name.startswith("Z")]
    uniform_direction_dist = sp.stats.uniform_direction(2)
    obstacle_direction = [[x, y, 0] for x,y in uniform_direction_dist.rvs(len(obstacles))]
    unused = np.zeros(1, dtype=np.int32)

    #Initialize Car Control
    CarPosition = d.qpos[:3]
    CarOrientation = d.qpos[3:7]

    wall_positions = [ (pos[0]*2, pos[1]*2) for pos, tile in tiles.items() if tile == "#" ]
    walls_nx2,walls_2xn = get_wall_obstacles(wall_positions)

    cylinder_obstacles = []

    CarControl = RacingCarControl(CarPosition,CarOrientation,[final_pos[0]*2, final_pos[1]*2],LocalController=6)
    CarControl.plan_globally(GlobalController=1,Walls=walls_2xn)

    plot_global_map(walls_2xn,CarPosition,final_pos,CarControl.waypoints)


    with mujoco.viewer.launch_passive(m, d, key_callback=mujoco_viewer_callback) as viewer:
      prev_waypoint_index = 1
      add_reference_path(viewer,CarControl.waypoints)

      viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
      viewer.cam.fixedcamid = m.camera("robot-third_person").id

      # These actuator names are defined in the model XML file for the robot.
      # Prefixes distinguish from other actuators from the same model.
      velocity = d.actuator("robot-throttle_velocity")
      steering = d.actuator("robot-steering")

      # Close the viewer automatically after 30 wall-clock-seconds.
      start = time.time()
      while viewer.is_running() and time.time() - start < 3000:
        step_start = time.time()

        if not paused:
            velocity.ctrl = CarControl.ControlVelocity
            steering.ctrl = CarControl.ControlAttitude

            cylinder_obstacles.clear()
            # Update obstables (bouncing movement)
            for i, x in enumerate(obstacles):
                

                dx = obstacle_direction[i][0]
                dy = obstacle_direction[i][1]

                px = m.geom_pos[x][0]
                py = m.geom_pos[x][1]
                pz = 0.02

                nearest_dist = mujoco.mj_ray(m, d, [px, py, pz], obstacle_direction[i], None, 1, -1, unused)

                if nearest_dist >= 0 and nearest_dist < 0.4:
                    obstacle_direction[i][0] = -dy
                    obstacle_direction[i][1] = dx

                m.geom_pos[x][0] = m.geom_pos[x][0]+dx*0.001
                m.geom_pos[x][1] = m.geom_pos[x][1]+dy*0.001

                cylinder_obstacles.append([m.geom_pos[x][0], m.geom_pos[x][1]])

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(m, d)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()
            
            #Update Controller
            CarPosition = d.qpos[:3]
            CarOrientation = d.qpos[3:7]

            CarControl.plan_locally(d,CarPosition[:2],CarOrientation,static_obstacles=walls_nx2,dynamic_obstacles=cylinder_obstacles)
            
            if(prev_waypoint_index != CarControl.current_idx and CarControl.current_idx != len(CarControl.waypoints)):
                geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
                mujoco.mjv_initGeom(
                geom,
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([0.1, 0.1, 0.1]),  # label_size
                pos=np.array([CarControl.waypoints[CarControl.current_idx][0],CarControl.waypoints[CarControl.current_idx][1],0.1]),  # label position
                mat=np.eye(3).flatten(),  # label orientation
                rgba=np.array([1, 0, 0, 1])  # for wapoint
                )
                viewer.user_scn.ngeom += 1
                prev_waypoint_index = CarControl.current_idx

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
          time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
