# dg-commons
from dg_commons import PlayerName
from dg_commons.planning import PolygonGoal
from dg_commons.sim import SimObservations
from dg_commons.sim.agents import Agent, NPAgent
from dg_commons.sim.models.obstacles import StaticObstacle
from dg_commons.sim.models.spacecraft import SpacecraftCommands
from dg_commons.sim.models.spacecraft_structures import SpacecraftGeometry, SpacecraftParameters

# misc
import sys, os
from typing import Sequence, Tuple
from shapely.geometry import Point
import numpy as np

import time
import math

# RRT*
from .RRTFamilyOfPlanners import SamplingBasedPathPlanner
from .SmallestEnclosingCircle import make_circle

# smooth local path
from .SmoothingPath import get_smoothed_path

# MPC
import do_mpc
from .MPCTracker import mpc_model, mpc_tracking


class Pdm4arAgent(Agent):
    """This is the PDM4AR agent.
    Do NOT modify the naming of the existing methods and the input/output types.
    Feel free to add additional methods, objects and functions that help you to solve the task"""

    def __init__(self,
                 goal: PolygonGoal,
                 static_obstacles: Sequence[StaticObstacle],
                 sg: SpacecraftGeometry,
                 sp: SpacecraftParameters):

        self.goal = goal
        self.static_obstacles = static_obstacles
        self.sg = sg
        self.sp = sp

        self.name = None
        self.iter = 0

        self.world = [item.shape for item in static_obstacles]
        self.bounds = self.world[0].coords
        self.stat_obstacles = self.world[1:]
        self.min_x, self.min_y = self.bounds[0]
        self.max_x, self.max_y = self.bounds[2]
        self.area_world = (self.max_x - self.min_x) * (self.max_y - self.min_y)
        self.boundaries = (self.min_x, self.min_y, self.max_x, self.max_y)
        self.goal_region = self.goal.get_plottable_geometry()

        self.object_radius = 1.4*max(sg.lf+sg.lr, 2*sg.w_half)
        self.steer_distance = 5 #2*(sg.lf+sg.lr) #abs( 0.1*((self.max_x-self.min_x)/2 + (self.max_y-self.min_y)/2) )
        
        self.resolution = 4
        self.drawResults = True
        self.runForFullIterations = True

        self.sbpp = SamplingBasedPathPlanner()

        self.start_planning = True

        self.n_horizon_long = 15
        self.n_horizon_longer = 30
        self.error_orientation_threshold = 0.35  # in radiants, error threshold to lengthen the MPC horizon
        self.error_position_threshold = 3 # in meters, error threshold to lengthen the MPC horizon
        
        self.avoidance_activation_radius = 20 # in meters, distance range from dynamic obstacles that activates the avoidance behavior
        self.extra_distance = 1.0 # extra distance to keep from obstacle, for more safety
        
    def find_path(self, start, RRT_algorithm = "Informed RRT*"):
        
        num_iterations = int(self.area_world) 
        print("Initialize", RRT_algorithm, ", margin from obstacles: ", self.object_radius, " m")
        self.sbpp.RRTStar(RRT_algorithm, self.stat_obstacles, self.boundaries, start, self.goal_region, self.object_radius, self.steer_distance, 
                                    None, num_iterations, self.resolution, self.runForFullIterations, self.drawResults)   

        while not self.sbpp.RRTFamilySolver.path:
            self.object_radius *= 0.9
            print("Re-initialize", RRT_algorithm, ", margin from obstacles: ", self.object_radius, " m")
            num_iterations = int(self.area_world)      
            del self.sbpp
            self.sbpp = SamplingBasedPathPlanner()
            self.sbpp.RRTStar(RRT_algorithm, self.stat_obstacles, self.boundaries, start, self.goal_region, self.object_radius, self.steer_distance, 
                                    None, num_iterations, self.resolution, self.runForFullIterations, self.drawResults)   

        print("Initial path found!")
        num_iterations = int(self.area_world/3)
        print("Refine ", RRT_algorithm)
        self.sbpp.RRTStar(RRT_algorithm, self.stat_obstacles, self.boundaries, start, self.goal_region, self.object_radius, self.steer_distance, 
                                    None, num_iterations, self.resolution, self.runForFullIterations, self.drawResults)                                   

        self.segmented_path = np.asarray(self.sbpp.RRTFamilySolver.path)
        self.segmented_path[0,:] = (self.segmented_path[0,:] + self.segmented_path[1,:])/2

    def check_avoidance_behavior(self, current_pose, sim_obs):

        start_avoidance = False
        avoidance_region = Point(current_pose).buffer(self.avoidance_activation_radius, 25)
        obstacles_keys = list(sim_obs.players.keys())[1:]
        min_dist_obstacle = float('inf')
        id_obstacle = -1
        dyn_obstacle_poses = []
        for obstacle in obstacles_keys:
            dyn_obstacle = sim_obs.players[obstacle].occupancy
            if dyn_obstacle.intersects(avoidance_region):
                start_avoidance = True
                obstacle_circle = make_circle(sim_obs.players[obstacle].occupancy.exterior.coords)
                X_obstacle, Y_obstacle, radius_obstacle = obstacle_circle[0], obstacle_circle[1], obstacle_circle[2]
                distance_to_obstacle = np.sqrt( (current_pose[0]-X_obstacle)**2 + (current_pose[1]-Y_obstacle)**2 )
                if min_dist_obstacle > distance_to_obstacle:
                    min_dist_obstacle = distance_to_obstacle
                    id_obstacle = obstacle
        
        if start_avoidance:
            dyn_obstacle = sim_obs.players[id_obstacle]
            obstacle_circle = make_circle(dyn_obstacle.occupancy.exterior.coords)
            X_obstacle, Y_obstacle, radius_obstacle = obstacle_circle[0], obstacle_circle[1], obstacle_circle[2]
            psi_obstacle, vx_obstacle, vy_obstacle, dpsi_obstacle = dyn_obstacle.state.psi, dyn_obstacle.state.vx, dyn_obstacle.state.vy, dyn_obstacle.state.dpsi
            dyn_obstacle_poses = [[X_obstacle, Y_obstacle]]
            # predict pose of obstacle in the next 40 steps (4 seconds)
            for i in range(40):
                vx_obstacle_temp = vx_obstacle
                vx_obstacle += 0.1*(dpsi_obstacle * vy_obstacle)
                vy_obstacle += 0.1*(-dpsi_obstacle * vx_obstacle_temp)
                psi_obstacle += 0.1*(dpsi_obstacle)
                v_X_obstacle = vx_obstacle * math.cos(psi_obstacle) - vy_obstacle * math.sin(psi_obstacle)
                v_Y_obstacle = vx_obstacle * math.sin(psi_obstacle) + vy_obstacle * math.cos(psi_obstacle)        
                next_X_obstacle = dyn_obstacle_poses[-1][0]+0.1*v_X_obstacle
                next_Y_obstacle = dyn_obstacle_poses[-1][1]+0.1*v_Y_obstacle
                dyn_obstacle_poses.append([next_X_obstacle, next_Y_obstacle])
            distance_to_obstacle_X = current_pose[0]-X_obstacle
            distance_to_obstacle_Y = current_pose[1]-Y_obstacle
            distance_to_obstacle = np.sqrt( distance_to_obstacle_X**2 + distance_to_obstacle_Y**2 )
            safety_distance = max(self.sg.lf,self.sg.lr) + radius_obstacle
            
            print("Dynamic obstacle ", id_obstacle, " at distance ", distance_to_obstacle, " m")

        else:
            safety_distance = None
            dyn_obstacle_poses = None

        return start_avoidance, safety_distance, dyn_obstacle_poses

    def on_episode_init(self, my_name: PlayerName):
        self.name = my_name

    def set_dynamic_obstacles(self, dyn_obs: NPAgent):
        self.dynamic_obstacles = dyn_obs

    def blockPrint(self):
        sys.stdout = open(os.devnull, 'w')

    def enablePrint(self):
        sys.stdout = sys.__stdout__
    
    def get_commands(self, sim_obs: SimObservations) -> SpacecraftCommands:
        """ This method is called by the simulator at each time step.

        This is how you can get your current state from the observations:
        my_current_state: SpacecraftState = sim_obs.players[self.name].state

        :param sim_obs:
        :return:
        """

        print("\n------------- ", self.iter/10, "s  -------------\n")

        # current state
        X = sim_obs.players[self.name].state.x
        Y = sim_obs.players[self.name].state.y
        psi = sim_obs.players[self.name].state.psi
        v_X = sim_obs.players[self.name].state.vx
        v_Y = sim_obs.players[self.name].state.vy
        d_psi = sim_obs.players[self.name].state.dpsi

        current_pose = (X,Y)

        if not self.start_planning:
            print("Velocity: x ", v_X, ", y ", v_Y)

        # RRT* PATH PLANNING

        # Create Informed RRT* graph at first iteration
        if self.start_planning:
            self.find_path(current_pose, "Informed RRT*")
            self.start_planning = False


        # MPC PATH TRACKING AND DYNAMIC OBSTACLES AVOIDANCE

        # check if avoidance behavior should be activated
        start_avoidance, safety_distance, dyn_obstacle_poses = self.check_avoidance_behavior(current_pose, sim_obs)        

        # smooth the local path around the spacecraft
        local_smoothed_path = get_smoothed_path(current_pose, self.segmented_path.tolist(), self.sp.vx_limits[1])

        # initialize MPC
        start_mpc_time = time.time()
        self.blockPrint()
        self.model = mpc_model(self.sg, start_avoidance)

        # compute position and orientation wrt the path to lenghten the MPC horizon if the current errors are too large
        n_avg = 5
        average_orientation = 0

        distance_to_path = np.sqrt( (X-local_smoothed_path[0][0])**2 + (Y-local_smoothed_path[0][1])**2 )

        for k in range(n_avg-1):
            dx_i = local_smoothed_path[k+1][0] - local_smoothed_path[k][0]
            dy_i = local_smoothed_path[k+1][1] - local_smoothed_path[k][1]
            theta_i = math.atan2(dy_i, dx_i);    # range [-pi, pi]
            average_orientation += theta_i / (n_avg-1)

        # Set the MPC
        if start_avoidance: 
            # start MPC with avoidance setting
            self.mpc = mpc_tracking(self.model, self.sg, self.sp, self.boundaries, local_smoothed_path, self.n_horizon_longer, True, safety_distance, self.extra_distance, dyn_obstacle_poses)
        elif ((abs(psi - average_orientation) > self.error_orientation_threshold) or (distance_to_path > self.error_position_threshold)):
            # if spaceshift has currently too much tracking error, start MPC with normal setting but with longer horizon
            self.mpc = mpc_tracking(self.model, self.sg, self.sp, self.boundaries, local_smoothed_path, self.n_horizon_longer, False)
        else:
            # start MPC with normal setting
            self.mpc = mpc_tracking(self.model, self.sg, self.sp, self.boundaries, local_smoothed_path, self.n_horizon_long, False)

        # Run the MPC
        self.mpc.x0 = np.array([X, Y, psi, v_X, v_Y, d_psi]).reshape(-1,1)
        self.mpc.set_initial_guess()
        # input for the spaceshift
        u0 = self.mpc.make_step(self.mpc.x0)
        acc_left, acc_right = u0[0][0], u0[1][0]

        self.enablePrint()

        print("Computation time mpc: ", time.time()-start_mpc_time, " s")

        self.iter += 1

        return SpacecraftCommands(acc_left, acc_right)
