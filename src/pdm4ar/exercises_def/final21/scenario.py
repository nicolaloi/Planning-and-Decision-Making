import random
from typing import Optional, Tuple

from commonroad.visualization.mp_renderer import MPRenderer
from dg_commons import apply_SE2_to_shapely_geo, X
from dg_commons.maps.shapely_viz import ShapelyViz
from dg_commons.maps.shapes_generator import create_random_starshaped_polygon
from dg_commons.planning import PlanningGoal, PolygonGoal
from dg_commons.sim.models.obstacles import StaticObstacle
from dg_commons.sim.models.spacecraft import SpacecraftState
from dg_commons.sim.scenarios.structures import DgScenario
from dg_commons.sim.simulator_visualisation import ZOrders
from geometry import SE2_from_xytheta
from matplotlib import pyplot as plt
from numpy import deg2rad
from shapely.geometry import Polygon, LinearRing

__all__ = ["get_dgscenario"]


def get_dgscenario(seed: Optional[int] = None) -> Tuple[DgScenario, PlanningGoal, X]:
    max_size = 100
    bounds = LinearRing([(0, 0), (0, max_size), (max_size, max_size), (max_size, 0), (0, 0)])
    shapes = [bounds]

    if seed is not None:
        random.seed(seed)

    # add some static obstacles to the environment
    # define the centre of the obstascles
    positions = [(70, 85), (15, 65), (30,31), (65,23), (75,50)]
    for pos in positions:
        # change size and shape
        poly = Polygon(create_random_starshaped_polygon(*pos, 8, 0.5, 0.5, 10))
        shapes.append(poly)

    obstacles = list(map(StaticObstacle, shapes))
    static_obstacles = dict(zip(range(len(obstacles)), obstacles))

    # initial states of the spacecraft
    x0 = SpacecraftState(x=9, y=4, psi=deg2rad(45), vx=5, vy=0, dpsi=0.02)
    
    # define goal region
    goal_poly = Polygon(((max_size, max_size), (max_size - 10, max_size),
                         (max_size - 10, max_size - 10), (max_size, max_size - 10)))
    goal = PolygonGoal(goal_poly)

    return DgScenario(static_obstacles=static_obstacles), goal, x0



