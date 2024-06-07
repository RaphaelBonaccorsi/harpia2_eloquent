#!/usr/bin/env python3

import sys, traceback

import math
import os
import pandas as pd
import pickle
import json
import rclpy
from rclpy.node import Node
import time
import logging

from std_msgs.msg import String
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
"""
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *
"""
from sensor_msgs.msg import BatteryState


from interfaces.msg import *
from mavros_msgs.msg import *
from interfaces.srv import *

# Used:
# WaypointList
# Waypoint
# PathPlanningResponse
# PathPlanning

import shapely.geometry

# Behaviours
from libs.Behaviours.behaviours import pulverize, picture

# Ray Casting
from libs.RayCasting.raycasting import point_in_polygon, Vector

# AG
from libs.AG.genetic import Subject, Genetic
from libs.AG.model import Mapa, Conversor, CartesianPoint
from libs.AG.visualization import vis_mapa

# RRT
from libs.RRT.rrt import RRT

# PFP
from libs.PotentialFieldPlanning.potential_field_planning import potential_field_planning

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Drone(Node):
    """
    A ROS 2 node representing a drone, capable of subscribing to battery state information.

    Attributes:
        sub_battery (Subscription): A subscription to the battery state topic.
        battery (float): The current battery percentage of the drone. None until the first message is received.

    Methods:
        battery_state_callback(data): Callback function for the battery state subscription. Updates the battery attribute with the latest battery percentage.
    """

    def __init__(self):
        """
        Initializes the Drone node, creates a subscription to the battery state topic.
        """
        super().__init__('drone_node')
        self.sub_battery = self.create_subscription(
            BatteryState,
            'mavros/battery',
            self.battery_state_callback,
            10)
        self.battery = None

    def battery_state_callback(self, data):
        """
        Callback function for the battery state subscription.

        Args:
            data (BatteryState): The message containing the battery state information.
        """
        self.battery = data.percentage * 100

# ---
# UTILS
def log(index):
    """
    Increments the replan count for a specified planner in the mission log.

    This function reads the current mission log from a JSON file, increments the replan count for the specified planner,
    and then writes the updated log back to the file. The specific planner is identified by the provided index.

    Args:
        index (int): The index of the planner in the 'knn' list within the last mission log entry. This index is used to
                     identify which planner's replan count to increment.

    Raises:
        FileNotFoundError: If the mission log file does not exist at the specified path.
        json.JSONDecodeError: If the mission log file is not a valid JSON file.
    """
    harpia_root_dir = get_harpia_root_dir()

    # log path
    log_path = os.path.join(harpia_root_dir, "results")
    log_json = os.path.join(log_path, "mission_log.json")

    # open log
    with open(log_json, "r") as log_file:
        log_data = json.load(log_file)

    # add replan
    log_data[-1]['knn'][index] += 1

    # dump log
    with open(log_json, 'w') as outfile:
        json.dump(log_data, outfile, indent=4)

def euclidean_distance(A, B):
    return math.sqrt((B.x - A.x) ** 2 + (B.y - A.y) ** 2)

def calc_dist_path(path):
    dist = 0
    i = 1
    for p in path:
        if p != path[-1]:
            A = CartesianPoint(p[0], p[1])
            B = CartesianPoint(path[i][0], path[i][1])
            dist += euclidean_distance(A, B)
            i += 1

    return dist

def wait_until(node: Node, check, msg=None, rate=1):
    """
    Waits until a given condition is met, periodically checking the condition.

    This function repeatedly checks a given condition by calling the provided `check` callable. It waits for the condition
    to return True, sleeping for a specified rate between checks. If a message is provided, it logs the message to the node's
    logger at each check. The function exits if the ROS context has been shut down.

    Args:
        node (Node): The ROS 2 node instance used for logging and creating a rate for sleeping between checks.
        check (callable): A callable that returns True when the condition to wait for is met, and False otherwise.
        msg (str, optional): A message to log at each check. If None, no message is logged. Defaults to None.
        rate (float, optional): The rate in Hz at which to check the condition and log the message. Defaults to 1.

    Returns:
        bool: True if the condition was met before ROS shutdown, False if ROS was shut down before the condition was met.
    """
    rate = node.create_rate(rate)

    while not check():
        if rclpy.ok() is False: return False
        if msg is not None: node.get_logger().info(msg)
        rate.sleep()

    return True

def get_harpia_root_dir():
    """
    Returns the root directory of the Harpia project.

    This function returns the absolute path of the Harpia project's root directory. The root directory is assumed to be located in the current user's home directory under the name 'harpia'.

    Returns
    -------
    str
        The absolute path of the Harpia project's root directory.
    """
    return os.path.expanduser("~/harpia") # Check if this is the correct path ********

def to_waypointList(route, geo_home):
    """
    Converts a list of Cartesian coordinates to a WaypointList with geographic coordinates.

    This function takes a route defined by Cartesian coordinates and a geographic home position, converts each Cartesian
    point to geographic coordinates, and then constructs a WaypointList object populated with these waypoints. Each waypoint
    is configured with predefined parameters suitable for navigation. The first waypoint in the list is marked as the current
    waypoint.

    Args:
        route (list of tuple): A list of tuples, where each tuple represents the Cartesian coordinates (x, y) of a waypoint.
        geo_home (GeoPoint): A GeoPoint object representing the geographic home position, used as the reference for converting
                             Cartesian coordinates to geographic coordinates.

    Returns:
        WaypointList: A WaypointList object populated with the converted waypoints, ready for use in navigation tasks.
    """
    geo_route = WaypointList()
    for wp in route:
        geo = Conversor.cart_to_geo(CartesianPoint(wp[0], wp[1]), geo_home)
        geo_wp = Waypoint()
        geo_wp.frame = 3
        geo_wp.command = 16
        if geo_route.waypoints == []:
            geo_wp.is_current = True
        else:
            geo_wp.is_current = False
        geo_wp.autocontinue = True
        geo_wp.param1 = 0
        geo_wp.param2 = 0
        geo_wp.param3 = 0
        geo_wp.param4 = 0
        geo_wp.x_lat = geo.latitude
        geo_wp.y_long = geo.longitude
        geo_wp.z_alt = 15
        geo_route.waypoints.append(geo_wp)

    return geo_route


def feasibility_ag(fitness_trace):
    """
    Determines the feasibility of a path based on its fitness trace.

    This function evaluates the feasibility of a path by examining its fitness trace, which contains metrics such as the
    number of obstacles hit and the final distance from the destination waypoint. The path is deemed 'infeasible' if it
    hits one or more obstacles, 'verify' if it ends more than 10 meters away from the destination waypoint, and 'feasible'
    otherwise.

    Args:
        fitness_trace (list): A list containing two elements. The first element (fitness_trace[0]) represents the final
                              distance from the destination waypoint in meters. The second element (fitness_trace[1])
                              represents the number of obstacles hit during the path.

    Returns:
        str: A string indicating the feasibility of the path. Possible values are 'infeasible', 'verify', or 'feasible'.
    """
    if fitness_trace[1] > 0:  # Hit obstacle (fit_obs) (max 1 hit)
        return "infeasible"
    elif fitness_trace[0] > 10:  # Away from destination wp (fit_d) (max 10 meters)
        return "verify"
    else:
        return "feasible"

def feasibility(route, obstacles, destination):
    """Checks if the route is feasible

    Args:
        route ([type]): [description]
        obstacles ([type]): [description]
        destination ([type]): [description]

    Returns:
        string: 'infeasible' - the route hits an obstacle
                'verify'     - the route does not arrive at the destination, according to min precision
                'feasible'   - the route is alright
    """

    # Minimum distance the last waypoint needs to be from the destination wp
    MIN_PRECISION = 10

    last_waypoint = Vector(route[-1][0], route[-1][1])
    logger.info(f"feasibility last_waypoint={last_waypoint}  destination={destination}")
    distance_to_objective = euclidean_distance(last_waypoint, destination)
    logger.info(f"feasibility distance_to_objective={distance_to_objective}")

    for obstacle in obstacles:
        for (x, y) in route:
            waypoint = Vector(x, y)
            if point_in_polygon(waypoint, obstacle):
                return "infeasible", distance_to_objective

    if distance_to_objective > MIN_PRECISION:
        return "verify", distance_to_objective

    return "feasible", distance_to_objective


def get_first_factible_route(ag):
    for subject in ag.history:
        feasibility_result = feasibility_ag(subject['fitness_trace'])

        if feasibility_result == "feasible":
            return subject['birth_time']

    return None

def pass_through_obstacle(obstacle_points, line_points):
    poly = shapely.geometry.Polygon(obstacle_points)
    line = shapely.geometry.LineString(line_points)
    
    return line.intersects(poly)

def count_obstacles(from_wp, to_wp, map, geo_home):
    obst_qty = 0
    line = Conversor.list_geo_to_cart([origin[:-1]] + [destination[:-1]], geo_home)

    for nfz in m['nfz']:
        poly = Conversor.list_geo_to_cart(nfz['geo_points'], geo_home)
        # poly1 = list_geo_to_cart(nfz['geo_points'], geo_home)
        if(pass_through_obstacle(poly, line)):
            obst_qty += 1

    return obst_qty
# ---
# PATH PLANNERS


def pfp(from_wp, to_wp, map, save_filename=None):
    """
    Performs path finding using a potential field algorithm from a starting waypoint to a destination waypoint.

    This function converts geographic coordinates of the starting and destination waypoints to Cartesian coordinates,
    constructs an obstacle list from no-fly zones (NFZs) defined in the map, and then uses a potential field planning
    algorithm to find a route. The feasibility of the found route is evaluated, and the route can be visualized by saving
    it to an image file if a filename is provided.

    Args:
        from_wp (Waypoint): The starting waypoint with geographic coordinates.
        to_wp (Waypoint): The destination waypoint with geographic coordinates.
        map (Map): The map object containing geographic information, including no-fly zones (NFZs).
        save_filename (str, optional): The filename for saving the visualization of the found route. If None, no file is saved.

    Returns:
        tuple: A tuple containing the following elements if a route is found:
            - A list of waypoints converted from the Cartesian coordinates of the found route.
            - The total distance of the path.
            - The feasibility result as a string ('feasible', 'infeasible', or 'verify').
            - The distance to the objective from the last point of the route.
            - The time taken to find the route.
            - The number of waypoints in the found route.
        If no route is found, None is returned.
    """
    origin = Conversor.geo_to_cart(from_wp.geo, map.geo_home)
    logger.info(f"origin={origin}")

    destination = Conversor.geo_to_cart(to_wp.geo, map.geo_home)
    logger.info(f"destination={destination}")

    # ---
    # obstacleList for check_collision_mode='ray_casting'
    obstacleList = [[Conversor.geo_to_cart(p.geo, map.geo_home) for p in area.points] for area in map.nfz]

    # Creates the ag_map in a structure the AG will understand
    rc_map = Mapa(
        origin,
        destination,
        areas_n=obstacleList,
    )
    # ---

    resolution = 20

    start_time = time.time()
    route = potential_field_planning(origin, destination, obstacleList, resolution)
    time_taken = time.time() - start_time

    if route is None:
        logger.warning("Cannot find route")
    else:
        logger.info("found route!!")
        logger.info(f"route={route}")

        feasibility_res, distance_to_objective = feasibility(route, obstacleList, destination)
        # Draw final route
        if save_filename:
            vis_mapa(rc_map, route=route, save=f"{save_filename}.png")

        return (
            to_waypointList(route, map.geo_home),
            calc_dist_path(route),
            feasibility_res,
            distance_to_objective,
            time_taken,
            len(route),
        )

def rrt(from_wp, to_wp, map, save_filename=None):
    """
    Performs path finding using the Rapidly-exploring Random Tree (RRT) algorithm from a starting waypoint to a destination waypoint.

    This function converts the geographic coordinates of the starting and destination waypoints to Cartesian coordinates
    relative to a specified home location. It then constructs a list of obstacles based on no-fly zones (NFZs) defined in
    the map. The RRT algorithm is applied to find a path from the origin to the destination, avoiding the defined obstacles.

    Args:
        from_wp (Waypoint): The starting waypoint with geographic coordinates.
        to_wp (Waypoint): The destination waypoint with geographic coordinates.
        map (Map): The map object containing geographic information, including no-fly zones (NFZs).
        save_filename (str, optional): The filename for saving the visualization of the found route. If None, no file is saved.

    Note:
        This function initializes the RRT path finding process by converting waypoints and obstacles to Cartesian coordinates
        and preparing the map for the RRT algorithm. The actual path finding logic and visualization saving are not implemented
        in this snippet.
    """
    origin = Conversor.geo_to_cart(from_wp.geo, map.geo_home)
    logger.info(f"origin={origin}")

    destination = Conversor.geo_to_cart(to_wp.geo, map.geo_home)
    logger.info(f"destination={destination}")

    obstacleList = [[Conversor.geo_to_cart(p.geo, map.geo_home) for p in area.points] for area in map.nfz]

    rc_map = Mapa(
        origin,
        destination,
        areas_n=obstacleList,
    )

    # Define the rand_area, based on origin and destination coordinates
    ps = [origin.x, origin.y, destination.x, destination.y]

    rand_area_x = math.floor(min(ps) * 1.2)
    rand_area_y = math.ceil(max(ps) * 1.2)
    rand_area = [rand_area_x, rand_area_y]

    logger.info(f"rand_area={rand_area}")

    dist_to_goal = euclidean_distance(origin, destination) * 1.2
    logger.info(f"dist_to_goal={dist_to_goal}")

    # Set Initial parameters
    rrt = RRT(
        start=[origin.x, origin.y],
        goal=[destination.x, destination.y],
        rand_area=rand_area,
        obstacle_list=obstacleList,
        expand_dis=25,
        path_resolution=1,
        goal_sample_rate=50,
        max_iter=5000,
        check_collision_mode="ray_casting",
    )

    start_time = time.time()
    route = rrt.planning(animation=False)

    if route is not None:
        route = list(reversed(route))

    time_taken = time.time() - start_time

    if route is None:
        logger.warning("Cannot find route")
        return None
    else:
        logger.info("found route!!")
        logger.info(f"route={route}")

        feasibility_res, distance_to_objective = feasibility(
            route, obstacleList, destination
        )

        # Draw final route
        if save_filename:
            vis_mapa(rc_map, route=route, save=f"{save_filename}.png")

        return (
            to_waypointList(route, map.geo_home),
            calc_dist_path(route),
            feasibility_res,
            distance_to_objective,
            time_taken,
            len(route),
        )


def ag(from_wp, to_wp, map, save_filename=None):
    """
    Executes path planning using a Genetic Algorithm (AG) from a starting waypoint to a destination waypoint.

    This function converts the geographic coordinates of the starting and destination waypoints to Cartesian coordinates,
    constructs an obstacle list from no-fly zones (NFZs) defined in the map, and initializes a Genetic Algorithm (AG) with
    parameters suitable for path planning. The AG attempts to find an optimal route from the origin to the destination,
    avoiding obstacles. The feasibility of the found route is evaluated, and the route can be visualized by saving it to an
    image file if a filename is provided.

    Args:
        from_wp (Waypoint): The starting waypoint with geographic coordinates.
        to_wp (Waypoint): The destination waypoint with geographic coordinates.
        map (Map): The map object containing geographic information, including no-fly zones (NFZs).
        save_filename (str, optional): The filename for saving the visualization of the found route. If None, no file is saved.

    Returns:
        tuple: A tuple containing the following elements if a route is found:
            - A list of waypoints converted from the Cartesian coordinates of the found route.
            - The total distance of the path.
            - The feasibility result as a string ('feasible', 'infeasible', or 'verify').
            - The distance to the objective from the last point of the route.
            - The time when the first feasible route was found during the AG execution.
            - The number of waypoints in the found route.
        If no route is found, None is returned.
    """
    # Convert map to ag_map
    nfzs = [[point.cartesian for point in area.points] for area in map.nfz]

    origin = Conversor.geo_to_cart(from_wp.geo, map.geo_home)
    destination = Conversor.geo_to_cart(to_wp.geo, map.geo_home)

    # Creates the ag_map in a structure the AG will understand
    ag_map = Mapa(
        origin,
        destination,
        areas_n=nfzs,
        inflation_rate=0.1,
    )

    ag = Genetic(
        Subject,
        ag_map,
        px0=ag_map.origin.x,
        py0=ag_map.origin.y,
        taxa_cross=5,
        population_size=20,
        max_exec_time=120,
        C_d=1000,
        C_obs=1000000,
        C_con=1,
        C_cur=10,
        C_t=10,
        C_dist=100,
        C_z_bonus=0,
        v_min=-3.0,
        v_max=3.0,
        e_min=-3,
        e_max=3,
        a_min=-3.0,
        a_max=3.0,
        T_min=5,
        T_max=15,
        delta_T=10,
        min_precision=10,
        mutation_prob=0.7,
        gps_imprecision=1,
    )

    best = ag.run(info=True)

    route = best.get_route()

    logger.info("Best route found:")

    # Check if route is feasible
    feasibility_res = feasibility_ag(best.fitness_trace)
    logger.info(f"Ag generated an <{feasibility_res}> route")

    distance_path = best.fitness_trace[5]  # (fit_dist)
    distance_to_objective = best.fitness_trace[0]  # (fit_d)

    first_factible_route_found_time = get_first_factible_route(ag)

    # Draw final route
    if save_filename:
        vis_mapa(ag_map, route=route, save=f"{save_filename}.png")

    return (
        to_waypointList(route, map.geo_home),
        distance_path,
        feasibility_res,
        distance_to_objective,
        first_factible_route_found_time,
        len(route),
    )

# ---
# SERVER

def select_planner(obstacles_qty, distance, battery):
    """Selects a path planner based on the information received

    Args:
        obstacles_qty (int): quantity of obstacles between the origin and the destination waypoint
        distance (float): the distance in a straight line between the origin and the destination waypoint

    Returns:
        selected_planner (str): the name of the selected planner, according to the intelligence
    """

    # return "rrt"

    global KNN

    # Predict the best planner
    return KNN.predict([[obstacles_qty, distance, battery]])

def run_path_planning(from_wp, to_wp, map, obstacles_qty):
    """
    Executes path planning between two waypoints using a selected algorithm based on obstacle quantity, distance, and UAV battery.

    This function calculates the Euclidean distance between the starting and destination waypoints, waits for the UAV battery
    status, and selects an appropriate path planning algorithm (either 'ag', 'rrt', or 'pfp') based on the number of obstacles,
    the distance, and the UAV's battery level. It attempts to find a feasible route using the selected algorithm. If the initial
    attempt is not feasible, it iterates through backup planners until a feasible route is found or all options are exhausted.

    Args:
        from_wp (Waypoint): The starting waypoint with geographic coordinates.
        to_wp (Waypoint): The destination waypoint with geographic coordinates.
        map (Map): The map object containing geographic information, including no-fly zones (NFZs).
        obstacles_qty (int): The quantity of obstacles to consider in the path planning process.

    Returns:
        list or None: A list of waypoints representing the found feasible route, or None if no feasible route is found.

    Note:
        The function uses a fallback mechanism, rotating through different planners ('ag', 'rrt', 'pfp') if the initially
        selected planner fails to find a feasible route. This process continues until a feasible route is found or all planners
        have been tried.
    """
    distance = euclidean_distance(from_wp.cartesian, to_wp.cartesian)

    uav = Drone()

    if not wait_until(lambda: uav.battery is not None, msg="Waiting for UAV battery..."):
        return None

    selected_planner = select_planner(obstacles_qty, distance, uav.battery)
    logger.info(f"selected_planner={selected_planner}")

    result = None

    if selected_planner == "ag":
        logger.info("Using ag")
        log(0)
        result = ag(from_wp, to_wp, map)

    elif selected_planner == "rrt":
        logger.info("Using rrt")
        log(1)
        result = rrt(from_wp, to_wp, map)

    elif selected_planner == "pfp":
        logger.info("Using pfp")
        log(2)
        result = pfp(from_wp, to_wp, map)

    else:
        logger.error("ERROR!!! Select or choose a different algorithm")
        return None

    if result is None:
        return None

    (route, _, feasibility_res, _, _, _) = result

    # Fica em loop chamando os diferentes planners ate encontrar uma rota "feasible"

    # sem aspas mesmo se quiser, reduzir para somente uma das opcoes, pra forçar algum planner
    # especifico, mas acho que assim, rotacionando tem mais chance de encontrar
    # uma rota em menos iterações

    backup_planners = [rrt, ag, pfp]
    i = 0
    while feasibility_res != 'feasible':
        result = backup_planners[i % 3](from_wp, to_wp, map)
        (route, _, feasibility_res, _, _, _) = result
        i += 1

    return route

class PathPlanningOp():
    PLAN_PATH     = 0
    PULVERIZE     = 1
    TAKE_PICTURE  = 2
    PLAN_PATH_RRT = 3

def path_planning(data):
    """
    Args:
        origin_point (RegionPoint): data.req.from
        destination_point (RegionPoint): data.req.to

    Returns:
        List[Waypoint]
    """

    route = WaypointList()

    # Organize inputs
    from_wp = data.r_from
    to_wp = data.r_to
    map = data.map

    if data.op == PathPlanningOp.PLAN_PATH:
        # OBSTACLES
        mapa_obstacles = pd.read_csv(f"{CSV_PATH}/mapa{data.map.id}_obstacles.csv", index_col=0)
        if data.name_from == "aux":
            obstacles_qty = 0  
        else:
            obstacles_qty = mapa_obstacles[data.name_from][data.name_to]

        route = run_path_planning(from_wp, to_wp, map, obstacles_qty)

    elif data.op == PathPlanningOp.PULVERIZE:
        route = pulverize(from_wp)

    elif data.op == PathPlanningOp.TAKE_PICTURE:
        distance = 5
        route = picture(from_wp, distance)

    elif data.op == PathPlanningOp.PLAN_PATH_RRT:
        result = rrt(from_wp, to_wp, map)
        (route, _, _, _, _, _) = result

    if route is None:
        # This will cause a `ServiceError` in rospy which is correct, since we
        # couldn't do the action.
        return None
    else:
        return PathPlanningResponse(route)

class PathPlanningServer(Node):
    def __init__(self):
        super().__init__('path_planning_server')
        self.srv = self.create_service(PathPlanning, 'harpia/path_planning', self.path_planning_callback)

        harpia_root_dir = get_harpia_root_dir()

        # Load KNN model
        knn_pickle_file = os.path.join(harpia_root_dir, "src/path_planning/scripts/libs/KNN/models/knn29.pickle")
        self.knn = pickle.load(open(knn_pickle_file, "rb"))

        # Configure CSV path
        self.csv_path = os.path.join(harpia_root_dir, "csv")

        self.get_logger().info("Path Planning Service Ready")

    def path_planning_callback(self, request, response):
        # Implement the service callback logic here
        # Use self.knn and self.csv_path as needed
        return response

def main(args=None):
    rclpy.init(args=args)
    path_planning_server = PathPlanningServer()
    rclpy.spin(path_planning_server)
    path_planning_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
