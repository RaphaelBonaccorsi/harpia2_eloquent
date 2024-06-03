#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException

# Brings in the SimpleActionClient
from actionlib_msgs.msg import GoalStatus
import sys, select

import math
import json
import time
import os
import argparse
from std_srvs.srv import Empty
#from rosplan_knowledge_msgs.srv import *
#from rosplan_knowledge_msgs.msg import *

from std_msgs.msg import String
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from interfaces.srv import *
from interfaces.msg import *

feedback = 0

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

def geo_to_cart(geo_point, geo_home):
    """
    Convert geographical coordinates to Cartesian coordinates.

    Args:
        geo_point (GeoPoint): Geographical coordinates.
        geo_home (GeoPoint): Home coordinates.

    Returns:
        Point: Cartesian coordinates.
    """

    def calc_y(lat, lat_):
        return (lat - lat_) * (10000000.0 / 90)

    def calc_x(longi, longi_, lat_):
        return (longi - longi_) * (
            6400000.0 * (math.cos(lat_ * math.pi / 180) * 2 * math.pi / 360)
        )

    x = calc_x(geo_point.longitude, geo_home.longitude, geo_home.latitude)
    y = calc_y(geo_point.latitude, geo_home.latitude)

    return Point(x, y, geo_point.altitude)

def file_check_id(fname):
    """
    Checks if a file exists and returns the next id and the file content.

    This function checks if a file exists at the given path. If the file exists, it opens the file, reads the JSON content, and calculates the next id by incrementing the id of the last entry in the file. If the file does not exist, it creates the file and returns 0 as the next id and an empty list as the file content.

    Parameters
    ----------
    fname : str
        The path of the file to check.

    Returns
    -------
    tuple
        A tuple containing two elements:
        - log_id (int): The next id to use.
        - log_file (list): The content of the file.
    """
    if os.path.exists(fname):
        with open(fname, "r") as log_file:
            log_file = json.load(log_file)

        log_id = log_file[-1]["id"] + 1

        return log_id, log_file
    else:
      print("Creating File.")
      open(fname, "w")
      
      return 0, []

def write_log(log, log_file):
    """
    Writes a log to a JSON file.

    This function opens a file in write mode and writes a log to it in JSON format. The JSON data is formatted with an indentation of 4 spaces. After writing the log, it closes the file.

    Parameters
    ----------
    log : dict
        The log to write to the file. It should be a dictionary that can be serialized to JSON.
    log_file : str
        The path of the file to write the log to.
    """
    with open(log_file, 'w') as outfile:
        json.dump(log, outfile, indent=4)
    outfile.close()


def total_goals(mission):
    """
    Counts the total number of goals in a mission.

    This function iterates over the "mission_execution" list in the given mission dictionary and increments a counter for each step, effectively counting the total number of goals in the mission.

    Parameters
    ----------
    mission : dict
        The mission dictionary. It should contain a "mission_execution" key with a list of steps as its value.

    Returns
    -------
    int
        The total number of goals in the mission.
    """
    total_goals = 0
    for step in mission["mission_execution"]:
        total_goals += 1
    return total_goals

def create_log(map, mission):
    """
    Creates a log for a mission and writes it to a JSON file.

    This function creates a log for a mission with various details such as the map id, mission id, total number of goals, and more. It then appends this log to a list of logs read from a JSON file and writes the updated list back to the file.

    Parameters
    ----------
    map : dict
        The map dictionary. It should contain an "id" key with the map id as its value.
    mission : dict
        The mission dictionary. It should contain an "id" key with the mission id as its value, and a "mission_execution" key with a list of steps as its value.

    """
    log = {}
    log_path = os.path.join(LOG_DIR, "mission_log.json")
    id_log, log_file = file_check_id(log_path)

    log['id']          = id_log
    log['map_id']      = map['id']
    log['mission_id']  = mission['id']
    log['total_goals'] = total_goals(mission)
    log['replan']      = 0
    log['knn']         = [0, 0, 0]
    log['cpu_time']    = []
    log['bn_net']      = []
    log['plans']       = []

    log_file.append(log)
    print(log['id'])

    write_log(log_file, log_path)

def feedback_callback(feedback_msg):
        # print(feedback_msg)
        global feedback
        feedback = feedback_msg.status
        # print('Received feedback: {}'.format(feedback))
        # return feedback

def test_client(node, hardware, map, mission, mission_file):
    """
    Creates a MissionPlannerGoal message from the hardware, map and mission
    data with op = 0. Sends the message to the action server
    "harpia/mission_goal_manager" and then publishes to "/harpia/mission" while
    the action server state is < 2.
    """

    # Creates the ActionClient, passing the type of the action
    client = ActionClient(node, MissionPlanner, 'harpia/mission_goal_manager')
    pub = node.create_publisher(Mission, "/harpia/mission", 10)

    create_log(map, mission)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MissionPlanner.Goal()
    goal.op = 0 # first add in the kwoledge base
    goal.mission = get_objects(hardware, map, mission)

    # Sends the goal to the action server.
    client.send_goal_async(goal)

    rate = node.create_rate(10)
    while client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
        pub.publish(goal.mission)

        if rclpy.ok():
            node.get_logger().warn("Shutting down test_client before mission completion")
            break

        rate.sleep()

    # Prints out the result of executing the action
    return client.get_result_async()

'''
    Hello again my brave friend,
    I know this entire code it's not ideal, but it'sa try,

   get_regions -> is an auxiliar function to create a list of a certain type of regions, as bases, nfz and roi read from the json file
   get_uav -> same as regions but for the uav
   get_map -> same as regions but for the map
   get_goals -> same as regions but for the goals

   get_objects -> call others functions to create a mission object

'''

def get_regions(tag, home):
    """
    Converts a list of regions from a map into a list of Region objects.

    This function iterates over the regions in a map with a given tag. For each region, it creates a Region object and sets its properties based on the region data. It also converts the region's center and points from geographic coordinates to cartesian coordinates relative to a home point. It appends each Region object to a list and returns this list.

    Parameters
    ----------
    tag : str
        The tag of the regions to get from the map.
    home : GeoPoint
        The home point for converting geographic coordinates to cartesian coordinates.

    Returns
    -------
    list
        A list of Region objects representing the regions in the map with the given tag.
    """
    region_list = []
    for r in map[tag]:
        region = Region()

        region.id = r["id"]
        region.name = r["name"]
        region.center.geo.latitude = r["center"][1]
        region.center.geo.longitude = r["center"][0]
        region.center.geo.altitude = r["center"][2]

        region.center.cartesian = geo_to_cart(region.center.geo, home)



        for p in r["geo_points"]:
            point = RegionPoint()

            point.geo.latitude = p[1]
            point.geo.longitude =p[0]
            point.geo.altitude = p[2]

            point.cartesian = geo_to_cart(point.geo, home)

            region.points.append(point)
        region_list.append(region)
    return region_list

def get_uav(hardware):
    """
    Creates a UAV object from a hardware dictionary.

    This function creates a UAV object and sets its properties based on the data in a hardware dictionary. The hardware dictionary should contain keys for 'id', 'name', 'camera', 'battery', 'frame', and 'input_capacity'. The 'camera', 'battery', and 'frame' keys should have values that are dictionaries with their own specific keys.

    Parameters
    ----------
    hardware : dict
        The hardware dictionary. It should contain the hardware configuration for the UAV.

    Returns
    -------
    UAV
        A UAV object with its properties set based on the hardware dictionary.
    """
    uav = UAV()
    uav.id = hardware['id']
    uav.name = hardware['name']

    uav.camera.name = hardware["camera"]['name']
    uav.camera.open_angle.x = hardware["camera"]['open_angle']['x']
    uav.camera.open_angle.y = hardware["camera"]['open_angle']['y']
    uav.camera.sensor.x = hardware["camera"]['sensor']['x']
    uav.camera.sensor.y = hardware["camera"]['sensor']['y']
    uav.camera.resolution.x = hardware["camera"]['resolution']['x']
    uav.camera.resolution.y = hardware["camera"]['resolution']['y']

    uav.camera.max_zoom = hardware["camera"]['max_zoom']
    uav.camera.mega_pixel = hardware["camera"]['mega_pixel']
    uav.camera.focus_distance = hardware["camera"]['focus_distance']
    uav.camera.shutter_time = hardware["camera"]['shutter_time']
    uav.camera.trigger = hardware["camera"]['trigger']
    uav.camera.weight = hardware["camera"]['weight']

    uav.battery.amp = hardware['battery']['amp']
    uav.battery.voltage = hardware['battery']['voltage']
    uav.battery.cells = hardware['battery']['cells']
    uav.battery.min = hardware['battery']['min']
    uav.battery.max = hardware['battery']['max']
    uav.battery.capacity = hardware['battery']['capacity']
    uav.battery.recharge_rate = hardware['battery']['recharge_rate']
    uav.battery.discharge_rate = hardware['battery']['discharge_rate']

    uav.frame.type = hardware['frame']['type']
    uav.frame.weight = hardware['frame']['weight']
    uav.frame.max_velocity = hardware['frame']['max_velocity']
    uav.frame.efficient_velocity = hardware['frame']['efficient_velocity']

    uav.input_capacity = hardware['input_capacity']

    return uav

def get_map(map):
    """
    Creates a Map object from a map dictionary.

    This function creates a Map object and sets its properties based on the data in a map dictionary. The map dictionary should contain keys for 'id', 'geo_home', 'roi', 'nfz', and 'bases'. The 'geo_home' key should have a value that is a list of geographic coordinates. The 'roi', 'nfz', and 'bases' keys should have values that are lists of regions. The function converts the 'geo_home' coordinates and the region points from geographic coordinates to cartesian coordinates relative to the home point.

    Parameters
    ----------
    map : dict
        The map dictionary. It should contain the map configuration.

    Returns
    -------
    Map
        A Map object with its properties set based on the map dictionary.
    """
    m = Map()
    m.id = map["id"]
    # mission.map.name =  map['name']

    home = GeoPoint()

    home.latitude = map["geo_home"][1]
    home.longitude = map["geo_home"][0]
    home.altitude = map["geo_home"][2]

    m.geo_home = home
    m.roi = get_regions("roi", home)
    m.nfz = get_regions("nfz", home)
    m.bases = get_regions("bases", home)

    return m

def get_goals(mission):
    """
    Converts a list of goals from a mission into a list of Goal objects.

    This function iterates over the "mission_execution" list in the given mission dictionary. For each goal in the list, it creates a Goal object and sets its action and region properties based on the goal data. It appends each Goal object to a list and returns this list.

    Parameters
    ----------
    mission : dict
        The mission dictionary. It should contain a "mission_execution" key with a list of goals as its value.

    Returns
    -------
    list
        A list of Goal objects representing the goals in the mission.
    """
    goals = []

    for g in mission["mission_execution"]:
        goal =  Goal()
        goal.action = g["command"]
        goal.region = g["instructions"]["area"]
        goals.append(goal) 

    return goals

def get_objects(hardware, map, goals):
    """
    Creates a Mission object from hardware, map, and goals dictionaries.

    This function creates a Mission object and sets its uav, map, and goals properties based on the data in the hardware, map, and goals dictionaries. It uses the get_uav, get_map, and get_goals functions to create the appropriate objects from the dictionaries.

    Parameters
    ----------
    hardware : dict
        The hardware dictionary. It should contain the hardware configuration for the UAV.
    map : dict
        The map dictionary. It should contain the map configuration.
    goals : dict
        The goals dictionary. It should contain the goals configuration for the mission.

    Returns
    -------
    Mission
        A Mission object with its properties set based on the hardware, map, and goals dictionaries.
    """
    mission = Mission()
    mission.uav = get_uav(hardware)
    mission.map = get_map(map)
    mission.goals = get_goals(goals)

    return mission

def parse_args():
    """
    Parses command-line arguments.

    This function uses argparse to parse command-line arguments. It expects three arguments: "mission_id", "map_id", and "hardware_id". These arguments should be integers representing the IDs of the mission, map, and hardware respectively.

    Returns
    -------
    Namespace
        An object that holds the parsed arguments as attributes. The object will have "mission_id", "map_id", and "hardware_id" attributes.
    """
    parser = argparse.ArgumentParser(description="Runs a mission with a map")

    parser.add_argument("mission_id" , type=int, help="Path to the json mission description")
    parser.add_argument("map_id"     , type=int, help="Path to the json map description")
    parser.add_argument("hardware_id", type=int, help="Path to the json hardware description")

    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()

    # Initializes a rclpy node so that the ActionClient can
    # publish and subscribe over ROS.
    rclpy.init(args=None)
    node = Node('test_client')

    harpia_root = get_harpia_root_dir()

    PATH    = os.path.join(harpia_root, "json/")
    LOG_DIR = os.path.join(harpia_root, "results/")

    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)

    #get file names
    map_filename     = os.path.join(PATH, "mapa.json")
    mission_filename = os.path.join(PATH, "missao.json")
    hw_filename      = os.path.join(PATH, "hardware.json")

    with open(mission_filename, "r") as mission_file:
        mission_file = json.load(mission_file)
        mission = mission_file[args.mission_id]

    with open(map_filename, "r") as map_file:
        map_file = json.load(map_file)
        map = map_file[args.map_id]

    with open(hw_filename, "r") as hw_file:
        hw_file = json.load(hw_file)
        hardware = hw_file[args.hardware_id]

    try:
        result = test_client(node, hardware, map, mission, mission_file)
        node.get_logger().info(f"Result: {result}")
    except ROSInterruptException:
        node.get_logger().error("program interrupted before completion")

    rclpy.shutdown()
