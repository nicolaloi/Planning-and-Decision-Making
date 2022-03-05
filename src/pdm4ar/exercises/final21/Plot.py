import itertools
from shapely.geometry import Point, MultiPoint, Polygon, LineString, box
from shapely import affinity
from math import sqrt
from matplotlib import pyplot as plt
from descartes import PolygonPatch

import yaml
import math


def plot_environment(env, bounds=None, figsize=None):
    if bounds is None and env.bounds:
        minx, miny, maxx, maxy = env.bounds
    elif bounds:
        minx, miny, maxx, maxy = bounds
    else:
        minx, miny, maxx, maxy = (-20,-20,20,20)

    max_width, max_height = 10, 10
    if figsize is None:
        width, height = max_width, (maxy-miny)*max_width/(maxx-minx)
        if height > 5:
            width, height = (maxx-minx)*max_height/(maxy-miny), max_height
        figsize = (width, height+1)
    #print(figsize)
    f = plt.figure(figsize=figsize)
    
    #f.hold('on')
    ax = f.add_subplot(111)
    for i, obs in enumerate(env):
        patch = PolygonPatch(obs, fc='purple', ec='purple', alpha=0.5, zorder=20)
        ax.add_patch(patch)

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    ax.set_aspect('equal', adjustable='box')
    plt.close(f)
    return ax

def plot_line(ax, line):
    x, y = line.xy
    ax.plot(x, y, color='gray', linewidth=1, solid_capstyle='round', zorder=1)


def plot_poly(ax, poly, color, alpha=1.0, zorder=3):
    patch = PolygonPatch(poly, fc=color, ec="purple", alpha=alpha, zorder=zorder)
    ax.add_patch(patch)



class Environment:
    def __init__(self, yaml_file=None, bounds=None):
        self.yaml_file = yaml_file
        self.environment_loaded = False
        self.obstacles = []
        self.obstacles_map = {}
        self.bounds = bounds
        if not yaml_file is None:
            if self.load_from_yaml_file(yaml_file):
                if bounds is None:
                    self.calculate_scene_dimensions()
                self.environment_loaded = True

    @property
    def bounds(self):
        return self.bounds

    def add_obstacles(self, obstacles):
        self.obstacles = self.obstacles + obstacles
        self.calculate_scene_dimensions()

    def calculate_scene_dimensions(self):
        """Compute scene bounds from obstacles."""
        points = []
        for elem in self.obstacles:
            points = points + list(elem.boundary.coords)

        mp = geom.MultiPoint(points)
        self.bounds = mp.bounds

    def load_from_yaml_file(self, yaml_file):
        f = open(yaml_file)
        self.data = yaml.safe_load(f)
        f.close()
        return self.parse_yaml_data(self.data)

    def parse_yaml_data(self, data):
        if 'environment' in data:
            env = data['environment']
            self.parse_yaml_obstacles(env['obstacles'])
            # self.parse_yaml_features(env['features'])
            return True
        else:
            return False

    def parse_yaml_obstacles(self, obstacles):
        self.obstacles = []
        self.obstacles_map = {}
        for name, description in obstacles.iteritems():
            # Double underscore not allowed in region names.
            if name.find("__") != -1:
                raise Exception("Names cannot contain double underscores.")
            if description['shape'] == 'rectangle':
                parsed = self.parse_rectangle(name, description)
            elif description['shape'] == 'polygon':
                parsed = self.parse_polygon(name, description)
            else:
                raise Exception("not a rectangle")
            if not parsed.is_valid:
                raise Exception("%s is not valid!"%name)
            self.obstacles.append(parsed)
            self.obstacles_map[name] = parsed
        self.expanded_obstacles = [obs.buffer(0.75/2, resolution=2) for obs in self.obstacles]

    
    def parse_rectangle(self, name, description):
        center = description['center']
        center = geom.Point((center[0], center[1]))
        length = description['length']
        width = description['width']
        # convert rotation to radians
        rotation = description['rotation']# * math.pi/180
        # figure out the four corners.
        corners = [(center.x - length/2., center.y - width/2.),
                   (center.x + length/2., center.y - width/2.),
                   (center.x + length/2., center.y + width/2.),
                   (center.x - length/2., center.y + width/2.)]
        # print(corners)
        polygon = geom.Polygon(corners)
        out = affinity.rotate(polygon, rotation, origin=center)
        out.name = name
        out.cc_length = length
        out.cc_width = width
        out.cc_rotation = rotation
        return out

    def parse_polygon(self, name, description):
        _points = description['corners']
        for points in itertools.permutations(_points):
            polygon = geom.Polygon(points)
            polygon.name = name
            if polygon.is_valid:
                return polygon

    def save_to_yaml(self, yaml_file):
        yaml_dict = {}
        obstacles = {}
        for i, ob in enumerate(self.obstacles):
            ob_dict = {}
            ob_dict['shape'] = 'polygon'
            ob_dict['corners'] = [list(t) for t in list(ob.boundary.coords)]
            ob_name = "obstacle%.4d"%i
            obstacles[ob_name] = ob_dict
        yaml_dict['environment'] = {'obstacles' : obstacles}
        
        f = open(yaml_file, 'w')
        f.write(yaml.dump(yaml_dict, default_flow_style=None))
        f.close()
    

def draw_results(algo_name, segmented_path, V, E, env, bounds, start_pose, goal_region):
    """
    Plots the path from start node to goal region as well as the graph (or tree) searched with the Sampling Based Algorithms.

    Args:
        algo_name (str): The name of the algorithm used (used as title of the plot)
        segmented_path (list<(float,float), (float,float)>): The sequence of coordinates traveled to reach goal from start node
        V (set<(float, float)>): All nodes in the explored graph/tree
        E (set<(float,float), (float, float)>): The set of all edges considered in the graph/tree
        env (yaml environment): 2D yaml environment for the path planning to take place
        bounds (int, int int int): min x, min y, max x, max y of the coordinates in the environment.
        start_pose(float,float): Coordinates of initial point of the path.
        goal_region (Polygon): A polygon object representing the end goal.

    Return:
        None

    Action:
        Plots a path using the environment module.
    """

    graph_size = len(V)
    path_size = len(segmented_path)
    # Calculate path length
    path_length = 0.0
    for i in range(path_size-1):
        path_length += euclidian_dist(segmented_path[i], segmented_path[i+1])
    print("Path length: ", path_length)
    # Create title with descriptive information based on environment, path length, and elapsed_time
    title = algo_name #+ "\n" + str(graph_size) + " Nodes. " + "Nodes visited: " + str(path_size) + "\n Path Length: " + str(round(path_length,2))

    # Plot environment
    env_plot = plot_environment(env, bounds)
    # Add title
    env_plot.set_title(title)
    # Plot goal
    plot_poly(env_plot, goal_region, '#f3d184')
    # Plot start
    buffered_start_vertex = Point(start_pose).buffer(1, 20)
    plot_poly(env_plot, buffered_start_vertex, 'red', zorder=2)

    # Plot Edges explored by ploting lines between each edge
    for edge in E:
        line = LineString([edge[0], edge[1]])
        plot_line(env_plot, line)

    # Plot segmented path
    plot_path(env_plot, segmented_path)
    
    env_plot.figure.savefig('RRT* Path.png')
    plt.close(env_plot.figure)


def euclidian_dist(point1, point2):
    return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)


def plot_path(env_plot, path):
    # Plots path by taking an enviroment plot and ploting in blue the edges that form part of the path
    line = LineString(path)
    x, y = line.xy
    env_plot.plot(x, y, color='blue', linewidth=3, solid_capstyle='round', zorder=1)



