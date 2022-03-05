from __future__ import division
from shapely.geometry import Point, LineString, Polygon
import random
import math
import numpy as np
from scipy import interpolate
from rtree import index
from .Plot import draw_results


class SamplingBasedPathPlanner():

    def __init__(self):

        self.RRTFamilySolver = RRTFamilyPathPlanner()

    def RRTStar(self, RRT_algorithm, environment, bounds, start_pose, goal_region, object_radius, steer_distance, distance_travelled, num_iterations, resolution=3, runForFullIterations=True, drawResults=False):
        """Returns a path from the start_pose to the goal region in the current environment using RRT*.

        Args:
            RRT_algorithm (string): A string representing what type of algorithm to use. Options are 'RRT*', and 'Informed RRT*'.
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches
            num_iterations (int): How many points are sampled for the creationg of the tree
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            runForFullIterations (bool): Optional, if False return the first path found without having to sample all num_iterations points.
            drawResults (bool): Optional, if set to True it plots the path and enviornment using a matplotlib plot.

        Returns:
            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the tree
        """

        self.RRTFamilySolver.getPath(RRT_algorithm, environment, bounds, start_pose, goal_region, object_radius, steer_distance, 
        distance_travelled, num_iterations, resolution, runForFullIterations)

        if self.RRTFamilySolver.path and drawResults:

            draw_results(RRT_algorithm, self.RRTFamilySolver.path, self.RRTFamilySolver.V, self.RRTFamilySolver.E, 
                        environment, bounds, start_pose, goal_region)

            # increase atificially the path at the end, to not have problem with the MPC horizon
            dx = self.RRTFamilySolver.path[-1][0] - self.RRTFamilySolver.path[-2][0]
            dy = self.RRTFamilySolver.path[-1][1] - self.RRTFamilySolver.path[-2][1]
            for i in range(50):
                new_point_x = self.RRTFamilySolver.path[-1][0] + dx/5
                new_point_y = self.RRTFamilySolver.path[-1][1] + dy/5
                self.RRTFamilySolver.path.append((new_point_x,new_point_y))

class RRTFamilyPathPlanner():

    def __init__(self):

        self.do_initialisation = True

    def initialise(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution):
        """Initialises the planner with information about the environment and parameters for the rrt path planers

        Args:
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches
            num_iterations (int): How many points are sampled for the creationg of the tree
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            runForFullIterations (bool): If False RRT and RRTStar return the first path found without having to sample all num_iterations points.

        Returns:
            None
        """
        #self.env = environment
        self.obstacles = environment
        self.dynamic_obstacle = environment[0]
        self.bounds = bounds
        self.minx, self.miny, self.maxx, self.maxy = bounds
        self.goal_region = goal_region
        self.start_pose = start_pose
        self.pose = start_pose
        
        self.resolution = resolution
        self.steer_distance = steer_distance
        self.goal_pose = (goal_region.centroid.coords[0])
        self.path_found = False
        self.final_point = None

        self.V = []
        self.E = set()
        self.child_to_parent_dict = dict()
        self.n_nodes = 0
        self.steps = 0

        p = index.Property()
        p.interleaved = True
        self.rtree = index.Index(properties = p)

        self.path = []
        self.path_length = float('inf')
        self.path_size = 0
        self.smoothed_path = []
    
        self.c_best = float('inf') # Max length we expect to find in our 'informed' sample space starts as infinite


    def getPath(self, RRT_algorithm, environment, bounds, start_pose, goal_region, object_radius, steer_distance, distance_travelled, num_iterations, resolution, runForFullIterations):

        """Returns a path from the start_pose to the goal region in the current environment using the specified RRT-variant algorithm.

        Args:
            RRT_algorithm (str): A string representing what type of algorithm to use. Options are 'RRT*', and 'Informed RRT*'.
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches
            num_iterations (int): How many points are sampled for the creationg of the tree
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            runForFullIterations (bool): If False RRT and RRTStar return the first path found without having to sample all num_iterations points.
            

        Returns:
            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the tree
        """

        self.N = num_iterations
        self.check_interval = min(100, int(self.N/3))
        self.runForFullIterations = runForFullIterations
        self.obj_radius = object_radius

        if (self.do_initialisation):
            self.initialise(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution)
            self.do_initialisation = False 

        # Define start and goal in terms of coordinates. The goal is the centroid of the goal polygon.
        x0, y0 = start_pose
        x1, y1 = goal_region.centroid.coords[0]
        start = (x0, y0)
        goal = (x1, y1)

        # Handle edge case where where the start is already at the goal
        if start == goal:
            self.path = [start, goal]
            self.V.append(start)         
            self.V.append(goal)
            self.n_nodes += 2
            self.E.union([(start, goal)])
        # There might also be a straight path to goal, consider this case before invoking algorithm
        elif self.isEdgeCollisionFree(start, goal):
            self.path = [start, goal]
            self.V.append(start)           
            self.V.append(goal)
            self.n_nodes += 2
            self.E.union([(start, goal)])
        # Run the appropriate RRT algorithm according to RRT_algorithm
        else:
            if RRT_algorithm == "RRT*":
                self.RRTStarSearch()
            elif RRT_algorithm == "Informed RRT*":
                self.InformedRRTStarSearch()
            else:
                # The RRT* flavour has no defined algorithm, therefore return None for all values
                return None, None, None, None

    def RRTStarSearch(self):
        """Returns path using RRTStar algorithm.

        Uses the same structure as RRTSearch, except there's an additional 'rewire' call when adding nodes to the tree.
        This can be seen as a way to optimise the branches of the subtree where the new node is being added.

        Returns:
            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the tree
        """
        # Code is very similar to RRTSearch, so for simplicity's sake only the main differences have been commented.
        tree_size = 0
        goal_centroid = self.goal_region.centroid.coords[0]

        if ( len(self.V) == 0 ):
            self.V.append(self.start_pose)
            self.rtree.insert(self.n_nodes, (self.start_pose[0], self.start_pose[1], self.start_pose[0], self.start_pose[1]))
            self.n_nodes += 1

        for i in range(self.N):
            #print("Iter n. ", i)

            #print("  sampling point")
            if(random.random()>=0.90):
                random_point = goal_centroid
            else:
                random_point = self.get_collision_free_random_point()

            
            #print("  nearest point")
            nearest_point = self.find_nearest_point(random_point)
            #print("  steering to point")
            new_point = self.steer(nearest_point, random_point)

            #print("  Check collision")
            #print("    free from collision: ", self.isEdgeCollisionFree(nearest_point, new_point))
            if self.isEdgeCollisionFree(nearest_point, new_point):
                # Find the nearest set of points around the new point
                #print("   near set")
                nearest_set = self.find_nearest_set(new_point)
                #print("   min point")
                min_point = self.find_min_point(nearest_set, nearest_point, new_point)
                #print("  add to graph")

                # the next 2 lines prevent the parent bug

                if (min_point == new_point):
                    continue


                self.V.append(new_point)
                self.rtree.insert(self.n_nodes, (new_point[0], new_point[1], new_point[0], new_point[1]))
                self.n_nodes += 1
                self.E.add((min_point, new_point))
                self.setParent(min_point, new_point)
                
                # Main difference between RRT and RRT*, modify the points in the nearest set to optimise local path costs.
                #print("    rewire")
                self.rewire(nearest_set, min_point, new_point)
                if (self.path_found == False):
                    if (self.isAtGoalRegion(new_point)):
                        self.path_found = True
                if (self.path_found):
                    if (self.isAtGoalRegion(new_point)):
                        self.final_point = new_point
                if (i%self.check_interval == 0 and self.path_found): #self.isAtGoalRegion(new_point):
                    if not self.runForFullIterations:
                        self.path, tree_size, self.path_size, self.path_length = self.find_path(self.start_pose, self.final_point)

                        break
                    else:
                        tmp_path, tmp_tree_size, tmp_path_size, tmp_path_length = self.find_path(self.start_pose, self.final_point)
                        if tmp_path_length < self.path_length:
                            self.path_length = tmp_path_length
                            self.path = tmp_path
                            tree_size = tmp_tree_size
                            self.path_size = tmp_path_size



    def InformedRRTStarSearch(self):
        """Returns path using informed RRTStar algorithm.

        Uses the same structure as RRTStarSearch, except that once a path is found, sampling is restricted to an ellipse
        containing the shortest path found.

        Returns:
            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the tree
        """
        # Code is very similar to RRTStarSearch, so for simplicity's sake only the main differences have been commented.
        tree_size = 0
        #goal_centroid = self.get_centroid(self.goal_region)
        goal_centroid = self.goal_region.centroid.coords[0]
        solution_set = set()

        if (len(self.V) == 0):
            self.V.append(self.start_pose)
            self.rtree.insert(self.n_nodes, (self.start_pose[0], self.start_pose[1], self.start_pose[0], self.start_pose[1]))
            self.n_nodes += 1


        self.c_best = self.path_length
        start_obj = Point(self.start_pose).buffer(self.obj_radius, self.resolution)  
        c_min = start_obj.distance(self.goal_region)
        x_center = np.matrix([[(self.start_pose[0] + self.goal_pose[0]) / 2.0],[(self.start_pose[1] + self.goal_pose[1]) / 2.0], [0]])
        a_1 = np.matrix([[(self.goal_pose[0] - self.start_pose[0]) / c_min],[(self.goal_pose[1] - self.start_pose[1]) / c_min], [0]])
        id1_t = np.matrix([1.0,0,0])
        M = np.dot(a_1, id1_t)
        U,S,Vh = np.linalg.svd(M, 1, 1)
        C = np.dot(np.dot(U, np.diag([1.0,1.0,  np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])), Vh)

        for i in range(self.N):
            # The main difference in this algorithm is that we limit our sample space.
            # Sample space is defined by c_best (our best path and the maximum path length inside the new space)
            # c_min (the distance between start and goal), x_center (midpoint between start and goal) and C
            # only c_best changes whenever a new path is found.

            if(random.random()>=0.90):
                random_point = goal_centroid
            else:
                random_point = self.sample(self.c_best, c_min, x_center, C)
            nearest_point = self.find_nearest_point(random_point)
            new_point = self.steer(nearest_point, random_point)
            free_collision = self.isEdgeCollisionFree(nearest_point, new_point)

            if free_collision:
                nearest_set = self.find_nearest_set(new_point)
                min_point = self.find_min_point(nearest_set, nearest_point, new_point)
                if (min_point == new_point):
                    continue

                self.V.append(new_point)
                self.rtree.insert(self.n_nodes, (new_point[0], new_point[1], new_point[0], new_point[1]))
                self.n_nodes += 1
                self.E.add((min_point, new_point))
                self.setParent(min_point, new_point)
                self.rewire(nearest_set, min_point, new_point)

                if (self.path_found == False):
                    at_goal_region = self.isAtGoalRegion(new_point)
                    if (at_goal_region):
                        self.path_found = True
                if (self.path_found):
                    at_goal_region = self.isAtGoalRegion(new_point)
                    if (at_goal_region):
                        self.final_point = new_point
                if (i%self.check_interval == 0 and self.path_found): #self.isAtGoalRegion(new_point):
                    if not self.runForFullIterations:
                        self.path, tree_size, self.path_size, self.path_length = self.find_path(self.start_pose, self.final_point)
                        break
                    else:
                        solution_set.add(new_point)
                        tmp_path, tmp_tree_size, tmp_path_size, tmp_path_length = self.find_path(self.start_pose, self.final_point)
                        if tmp_path_length < self.path_length:
                            self.path_length = tmp_path_length
                            self.path = tmp_path
                            tree_size = tmp_tree_size
                            self.path_size = tmp_path_size
                            self.c_best = tmp_path_length # c_best is calculated everytime a path is found. Affecting the sample space.

    def is_path_invalid(self, path):
        for k in range(len(path) - 1):
            if not self.isEdgeCollisionFree(path[k], path[k+1]):
                return True


    def sample(self, c_max, c_min, x_center, C):
        
        if c_max < float('inf'):
            
            r = [c_max /2.0, math.sqrt(c_max**2 - c_min**2)/2.0, math.sqrt(c_max**2 - c_min**2)/2.0]
            L = np.diag(r)
            while True:
                x_ball = self.sample_unit_ball()
                point = np.dot(np.dot(C,L), x_ball) + x_center
                point = (point[(0,0)], point[(1,0)])
                # Pick a point, if no obstacle overlaps with a circle centered at point with some obj_radius then return said point.
                buffered_point = Point(point).buffer(self.obj_radius, self.resolution)
                if self.isPointCollisionFree(buffered_point):
                    return point
            
        else:
            point = self.get_collision_free_random_point()
        return point

    def sample_unit_ball(self):
        a = random.random()
        b = random.random()

        if b < a:
            tmp = b
            b = a
            a = tmp
        sample = (b*math.cos(2*math.pi*a/b), b*math.sin(2*math.pi*a/b))
        return np.array([[sample[0]], [sample[1]], [0]])

    def find_nearest_set(self, new_point, distance_travelled = None):
        points = set()
        if distance_travelled is None:
            ball_radius = self.find_ball_radius()
        else:
            ball_radius = distance_travelled

        idx_list = list(self.rtree.intersection(
            (new_point[0]-ball_radius, new_point[1]-ball_radius, new_point[0]+ball_radius, new_point[1]+ball_radius)))

        for idx in idx_list:
            points.add(self.V[idx])

        return points

    def find_ball_radius(self):
        unit_ball_volume = math.pi
        n = len(self.V)
        dimensions = 2.0
        gamma = (2**dimensions)*(1.0 + 1.0/dimensions) * (self.maxx - self.minx) * (self.maxy - self.miny)
        ball_radius = min(((gamma/unit_ball_volume) * math.log(n) / n)**(1.0/dimensions), self.steer_distance)
        return ball_radius


    def find_min_point(self, nearest_set, nearest_point, new_point):
        min_point = nearest_point        
        min_cost = self.cost(nearest_point) + self.linecost(nearest_point, new_point)
        for vertex in nearest_set:
            if self.isEdgeCollisionFree(vertex, new_point):
                temp_cost = self.cost(vertex) + self.linecost(vertex, new_point)
                if temp_cost < min_cost:
                    min_point = vertex
                    min_cost = temp_cost
        return min_point

    def rewire(self, nearest_set, min_point, new_point):
        # Discards edges in the nearest_set that lead to a longer path than going through the new_point first
        # Then add an edge from new_point to the vertex in question and update its parent accordingly.
        for vertex in nearest_set - set([min_point]):
            if self.isEdgeCollisionFree(vertex, new_point):
                if self.cost(vertex) > self.cost(new_point) + self.linecost(vertex, new_point):
                    parent_point = self.getParent(vertex)
                    self.E.discard((parent_point, vertex))
                    self.E.discard((vertex, parent_point))
                    self.E.add((new_point, vertex))
                    self.setParent(new_point, vertex)
        


    def cost(self, vertex):
        path, tree_size, path_size, path_length = self.find_path(self.start_pose, vertex)
        return path_length


    def linecost(self, point1, point2):
        return self.euclidian_dist(point1, point2)

    def getParent(self, vertex):
        return self.child_to_parent_dict[vertex]

    def setParent(self, parent, child):
        self.child_to_parent_dict[child] = parent

    def get_random_point(self):
        x = self.minx + random.random() * (self.maxx - self.minx)
        y = self.miny + random.random() * (self.maxy - self.miny)
        return (x, y)

    def get_collision_free_random_point(self):
        # Run until a valid point is found
        while True:
            point = self.get_random_point()
            # Pick a point, if no obstacle overlaps with a circle centered at point with some obj_radius then return said point.
            buffered_point = Point(point).buffer(self.obj_radius, self.resolution)
            if self.isPointCollisionFree(buffered_point):
                return point

    def isPointCollisionFree(self, point):
        for obstacle in self.obstacles:
            if obstacle.contains(point):
                return False
        return True

    def find_nearest_point(self, random_point):
        closest_point = None
        min_dist = float('inf')

        idx_closest_point = list(self.rtree.nearest((random_point[0], random_point[1], random_point[0], random_point[1]), 1))[0]
        closest_point = self.V[idx_closest_point]

        return closest_point

    def isOutOfBounds(self, point):
        if((point[0] - self.obj_radius) < self.minx):
            return True
        if((point[1] - self.obj_radius) < self.miny):
            return True
        if((point[0] + self.obj_radius) > self.maxx):
            return True
        if((point[1] + self.obj_radius) > self.maxy):
            return True
        return False


    def isEdgeCollisionFree(self, point1, point2):

        check_collision = set()
            
        if self.isOutOfBounds(point2):
            return False
        line = LineString([point1, point2])
        expanded_line = line.buffer(self.obj_radius, self.resolution)
        for obstacle in self.obstacles:
            if expanded_line.intersects(obstacle):
                return False
        return True

    def steer(self, from_point, to_point):
        fromPoint_buffered = Point(from_point).buffer(self.obj_radius, self.resolution)
        toPoint_buffered = Point(to_point).buffer(self.obj_radius, self.resolution)
        if fromPoint_buffered.distance(toPoint_buffered) < self.steer_distance:
            return to_point
        else:
            from_x, from_y = from_point
            to_x, to_y = to_point
            theta = math.atan2(to_y - from_y, to_x- from_x)
            new_point = (from_x + self.steer_distance * math.cos(theta), from_y + self.steer_distance * math.sin(theta))
            return new_point

    def isAtGoalRegion(self, point):
        buffered_point = Point(point).buffer(self.obj_radius, self.resolution)
        intersection = buffered_point.intersection(self.goal_region)
        inGoal = intersection.area / buffered_point.area
        return inGoal >= 0.5

    def euclidian_dist(self, point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    def find_path(self, start_point, end_point):
        # Returns a path by backtracking through the tree formed by one of the RRT algorithms starting at the end_point until reaching start_node.
        path = [end_point]
        tree_size, path_size, path_length = len(self.V), 1, 0
        current_node = end_point
        previous_node = None
        target_node = start_point

        while current_node != target_node:   
            parent = self.getParent(current_node)
            path.append(parent)
            previous_node = current_node
            current_node = parent
            path_length += self.euclidian_dist(current_node, previous_node)
            path_size += 1
        path.reverse()
        return path, tree_size, path_size, path_length

    def get_centroid(self, region):
        centroid = region.centroid.wkt
        filtered_vals = centroid[centroid.find("(")+1:centroid.find(")")]
        filtered_x = filtered_vals[0:filtered_vals.find(" ")]
        filtered_y = filtered_vals[filtered_vals.find(" ") + 1: -1]
        (x,y) = (float(filtered_x), float(filtered_y))
        return (x,y)
