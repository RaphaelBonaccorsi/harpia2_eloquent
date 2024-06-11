import os
import joblib
import rclpy
from rclpy.node import Node
from rclpy.rate import Rate
import sys
import math
import json
import time
import os
import psutil
from std_srvs.srv import Empty
"""
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *
"""
from diagnostic_msgs.msg import KeyValue

from std_msgs.msg import String

from interfaces.msg import ChangeMission, MissionPlannerAction, MissionPlannerGoal, MissionPlannerFeedback, MissionPlannerResult, Mission, MissionPlannerActionGoal
from interfaces.srv import PathPlanning

from mavros_msgs.msg import *
from mavros_msgs.srv import *

from sensor_msgs.msg import *
from geographic_msgs.msg import *
from geometry_msgs.msg import *

from actionlib_msgs.msg import GoalStatusArray, GoalID

KB_UPDATE_ADD_KNOWLEDGE = 0
KB_UPDATE_RM_KNOWLEDGE = 2
KB_UPDATE_ADD_GOAL = 1
KB_UPDATE_RM_GOAL = 3
KB_UPDATE_ADD_METRIC = 4

KB_ITEM_INSTANCE = 0
KB_ITEM_FACT = 1
KB_ITEM_FUNCTION = 2
KB_ITEM_EXPRESSION = 3
KB_ITEM_INEQUALITY = 4

OP_UPDATE_KNOWLEDGE_BASE = 0
OP_REPLAN                = 1
OP_ADD_RM_GOALS          = 2



absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)


class Drone(Node):
    def __init__(self):
        super().__init__('drone')
        self.sub_position = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.global_position_callback, 10)
        self.sub_battery  = self.create_subscription(BatteryState, 'mavros/battery', self.battery_state_callback, 10)
        self.sub_mission  = self.create_subscription(WaypointReached, 'mavros/mission/reached', self.reached_callback, 10)
        self.latitude     = None
        self.longitude    = None
        self.battery      = None
        self.current      = None

    def global_position_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def battery_state_callback(self, data):
        self.battery = data.percentage * 100

    def reached_callback(self, data):
        self.current = data.wp_seq + 1

class Mission(Node):

    def __init__(self):
        super().__init__('mission')
        self.mission_sub = self.create_subscription(MissionPlannerActionGoal, '/harpia/mission_goal_manager/goal', self.mission_callback, 10)
        self.mission = None

    def mission_callback(self, data):
        self.mission = data.goal.mission
        
    def get_mission(self):
        return self.mission


def wait_until(check, msg=None, rate=1):
    rate = rclpy.Rate(rate)

    while not check():
        if rclpy.ok(): return False
        if msg is not None: self.get_logger().info(msg)
        rate.sleep()

    return True

def find_at(map, goals, latitude, longitude):
    """
    Finds a base that is withing a 50m radius. If none is found, try to find a region which has a center
    within the same range. If none is found, return `None`. If there is one of these waypoints in range,
    the drone is considered at that waypoint.
    """

    #get current lat long
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    for base in map.bases:
        d = euclidean_distance(base.center.cartesian, cart_location)
        # rospy.loginfo(f"base = {base.name} distance={d}")
        if d <= 50:
            return base

    region_names = { g.region for g in goals }

    for region in map.roi:
        if region.name in region_names:
            d = euclidean_distance(cart_location, region.center.cartesian)
            # rospy.loginfo(f"base = {region.name} distance={d}")
            if d <= 50:
                return region

    return None


def find_nearest_base(map, latitude, longitude):
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    return min(map.bases, key=lambda base: euclidean_distance(base.center.cartesian, cart_location))

class PathPlanningClient(Node):
    def __init__(self):
        super().__init__('path_planning_client')
        self.cli = self.create_client(PathPlanning, "/harpia/path_planning")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PathPlanning.Request()

    def call_path_planning(self, r_from, r_to, map):
        self.req.start = r_from.center
        self.req.goal = r_to.center
        self.req.start_region = r_from.name
        self.req.goal_region = r_to.name
        self.req.algorithm = 3
        self.req.map = map
        self.future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            return self.future.result()
        else:
            self.get_logger().error("Service call failed %r" % (self.future.exception(),))
            return None
'''
    Callers for MAVRos Services
'''

def mavros_cmd(node: Node, topic: str, msg_ty: type, error_msg="MAVROS command failed: ", **kwargs):
    cli = node.create_client(msg_ty, topic)
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    req = msg_ty.Request(**kwargs)
    future = cli.call_async(req)

    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(future.result())
    else:
        node.get_logger().error(f"{error_msg} {future.exception()}")

def land():
    mavros_cmd(
        '/mavros/cmd/land',
        CommandTOL,
        error_msg="Landing failed",
        altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
    )

def set_mode(mode):
    mavros_cmd(
        '/mavros/set_mode',
        SetMode,
        error_msg="Set mode failed",
        custom_mode=mode
    )

def clear_mission():
    mavros_cmd(
        '/mavros/mission/clear',
        WaypointClear,
        error_msg="Clear mission failed"
    )

def send_route(route):
    mavros_cmd(
        '/mavros/mission/push',
        WaypointPush,
        error_msg="Send route failed",
        start_index=route.current_seq,
        waypoints=route.waypoints
    )

class UAVAction(Node):
    def __init__(self):
        super().__init__('uav_action')

    def go_to_base(self):
        """
        Go to nearest base immediately and land.
        """
        uav = Drone()
        mission_obj = Mission()
        mission = mission_obj.get_mission()
        print(mission)

        rate = Rate(1)

        if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."):
            self.get_logger().error("CRITICAL - MissionGoalManager was terminated while going to nearest base and will not finish the action")
            return

        at = find_at(mission.map, mission.goals, uav.latitude, uav.longitude)
        base = find_nearest_base(mission.map, uav.latitude, uav.longitude)

        # Create an instance of PathPlanningClient
        path_planning_client = PathPlanningClient()

        # Call the call_path_planning method
        route = path_planning_client.call_path_planning(at, base, mission.map)

        # send route to uav
        clear_mission(self)
        self.get_logger().info("Send Route")
        send_route(route.waypoints)

        # set mode to mission
        self.get_logger().info("Set Mode")
        set_mode("AUTO.MISSION")

        # wait to arrive.
        uav.current = 0
        while uav.current < len(route.waypoints.waypoints):
            rclpy.sleep(1)

        # land
        land(self)
        rclpy.sleep(30)

        return base