#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
import sys
import math
import json
import time
import os
import psutil
from std_srvs.srv import Empty
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

# Importação para mensagens do PlanSys2
from plansys2_msgs.msg import *
from plansys2_msgs.action import *

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
OP_REPLAN				 = 1
OP_ADD_RM_GOALS			 = 2


'''
	Classes to subscribe and publish services
'''

class Drone(object):
	"""
    A class to represent a drone.

    This class provides methods to handle the drone's global position, battery state, and mission status.

    Attributes
    ----------
    sub_position : rclpy.Subscription
        A ROS subscription to the 'mavros/global_position/global' topic.
    sub_battery : rclpy.Subscription
        A ROS subscription to the 'mavros/battery' topic.
    sub_mission : rclpy.Subscription
        A ROS subscription to the 'mavros/mission/reached' topic.
    latitude : float
        The current latitude of the drone.
    longitude : float
        The current longitude of the drone.
    battery : float
        The current battery percentage of the drone.
    current : int
        The current waypoint sequence number in the drone's mission.

    Methods
    -------
    global_position_callback(data: NavSatFix)
        Updates the drone's latitude and longitude based on a NavSatFix message.
    battery_state_callback(data: BatteryState)
        Updates the drone's battery percentage based on a BatteryState message.
    reached_callback(data: WaypointReached)
        Updates the drone's current waypoint sequence number based on a WaypointReached message.
    """
	def __init__(self):
		self.sub_position = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.global_position_callback)
		self.sub_battery  = self.create_subscription(BatteryState, 'mavros/battery', self.battery_state_callback)
		self.sub_mission  = self.create_subscription(WaypointReached, 'mavros/mission/reached', self.reached_callback)
		self.latitude	  = None
		self.longitude	  = None
		self.battery	  = None
		self.current	  = None

	def global_position_callback(self, data):
		"""
        Updates the drone's latitude and longitude based on a NavSatFix message.

        Parameters
        ----------
        data : NavSatFix
            The NavSatFix message received from the 'mavros/global_position/global' topic.
        """
		self.latitude = data.latitude
		self.longitude = data.longitude

	def battery_state_callback(self, data):
		"""
        Updates the drone's battery percentage based on a BatteryState message.

        Parameters
        ----------
        data : BatteryState
            The BatteryState message received from the 'mavros/battery' topic.
        """
		self.battery = data.percentage * 100

	def reached_callback(self, data):
		"""
        Updates the drone's current waypoint sequence number based on a WaypointReached message.

        Parameters
        ----------
        data : WaypointReached
            The WaypointReached message received from the 'mavros/mission/reached' topic.
        """
		self.current = data.wp_seq + 1

class ActionServer():
	"""
    A class to represent an action server for mission goal management.

    This class provides methods to handle mission goals, including receiving new goals, updating the knowledge base, replanning, and adding/removing goals.

    Attributes
    ----------
    a_server : rclpy.action.ActionServer
        The action server.
    uav : Drone
        The drone object.
    new_goals : list
        The new goals for the mission.
    Mission_Sub : rclpy.Subscription
        A ROS subscription to the '/harpia/mission_goal_manager/goal' topic.
    Goals_Sub : rclpy.Subscription
        A ROS subscription to the '/harpia/ChangeMission' topic.
    mission : Mission
        The current mission.
    change_goals : int
        The operation to perform on the goals (add or remove).

    Methods
    -------
    mission_callback(data: MissionPlannerActionGoal)
        Updates the current mission based on a MissionPlannerActionGoal message.
    new_goal_callback(data: ChangeMission)
        Updates the operation and new goals based on a ChangeMission message.
    run()
        Initiates the action server and blocks while the server is online.
    register_cancel_callback(cancel_cb: Callable)
        Registers a callback function to be called when a cancel request is received.
    abort()
        Aborts the current goal.
    succeed()
        Indicates that the current goal has been achieved.
    execute_cb(data: MissionPlannerGoal)
        Executes the requested operation based on a MissionPlannerGoal message.
    """

	def __init__(self):
		self.a_server = ActionServer(
			self,
			"harpia/mission_goal_manager",
			MissionPlannerAction,
			execute_cb=self.execute_cb,
			auto_start=False
		)
		self.uav = Drone()
		self.new_goals    = None
		self.Mission_Sub  = self.create_subscription(
			MissionPlannerActionGoal,
			'/harpia/mission_goal_manager/goal',
			self.mission_callback,
			10
		)
		self.Goals_Sub = self.create_subscription(
            ChangeMission,
            '/harpia/ChangeMission',
            self.new_goal_callback,
            10
        )
		self.mission      = None
		self.change_goals = None

	def mission_callback(self, data):
		"""
		Updates the current mission based on a MissionPlannerActionGoal message.

		Parameters
		----------
		data : MissionPlannerActionGoal
			The MissionPlannerActionGoal message received from the '/harpia/mission_goal_manager/goal' topic.
		"""
		self.mission = data.goal.mission

	def new_goal_callback(self, data):
		"""
		Updates the operation and new goals based on a ChangeMission message.

		Parameters
		----------
		data : ChangeMission
			The ChangeMission message received from the '/harpia/ChangeMission' topic.
		"""
		try:
			# print(data)
			self.change_goals = data.op
			self.new_goals = data.goals

		except:
			self.change_goals = 0

	def run(self):
		"""
		Initiates the action server and blocks while the server is online.
		"""
		rclpy.init()
		node = rclpy.create_node("mission_goal_manager_server")
		self.a_server.start()
		node.get_logger().info("Mission Goal Manager Service Ready")
		rclpy.spin(node)

	def register_cancel_callback(self,cancel_cb):
		"""
		Registers a callback function to be called when a cancel request is received.

		Parameters
		----------
		cancel_cb : Callable
			The callback function to register.
		"""
		self.a_server.set_aborted(MissionPlannerResult())

	def abort(self):   self.a_server.set_aborted(MissionPlannerResult())
	def succeed(self): self.a_server.set_succeeded(MissionPlannerResult())

	def execute_cb(self, data):
		"""
		Executes the requested operation based on a MissionPlannerGoal message.

		Parameters
		----------
		data : MissionPlannerGoal
			The MissionPlannerGoal message received from the action server.
		"""
		self.get_logger().info(f"EXECUTE - MissionGoalManager with op {data.op}")

		if data.op == OP_UPDATE_KNOWLEDGE_BASE:
			update_knowledge_base(data.mission.uav, data.mission.map, data.mission.goals, "base_1")
		elif data.op == OP_REPLAN:
			self.get_logger().info("REPLAN - MissionGoalManager")
			replan(data.mission)
		elif data.op == OP_ADD_RM_GOALS:
			self.get_logger().error("ADD/RM GOALS - MissionGoalManager not implemented yet")
			self.abort()
			return

		feedback_msg = MissionPlannerFeedback()
		

		print(self.a_server)
		while not call_mission_planning():
			if self.change_goals:
				print('MISSION GOAL Manager CANCEL')
				feedback_msg.status = 1				

				# while self.change_goals:
				self.a_server.publish_feedback(feedback_msg)

				#def replan(mission, uav, base, goals, op):
				replan(self.mission, self.uav, None, self.new_goals, self.change_goals)


			elif not wait_until(lambda: self.uav.battery is not None, msg="Waiting for UAV battery..."):
				# If the wait has failed abor the mission immediately
				self.abort()
				return

			if self.uav.battery <= 20:
				base = go_to_base(data.mission, self.uav)
				replan(self.mission, self.uav, base, None, 0)
			else:
				replan(data.mission, self.uav, None, None, 0)
			feedback_msg.status = 0
			self.a_server.publish_feedback(feedback_msg)
			self.change_goals = 0

		# Land the drone
		land()
		time.sleep(30)
		self.succeed()
#############################
def wait_until(check, msg=None, rate=1):
	"""
    Waits until a condition is met.

    This function repeatedly checks a condition at a specified rate and optionally logs a message until the condition is met or the ROS node is shut down.

    Parameters
    ----------
    check : Callable[[], bool]
        A function with no arguments that returns a boolean. The function will wait until this function returns True.
    msg : str, optional
        A message to log at each check if the condition is not met. If None, no message is logged. Default is None.
    rate : int, optional
        The rate at which to check the condition, in Hz. Default is 1.

    Returns
    -------
    bool
        True if the condition was met, False if the ROS node was shut down before the condition was met.
    """
	rate = rclpy.Rate(rate)

	while not check():
		if rclpy.ok(): return False # Need to test, not sure if the rclpy.ok have the same functionality as in ROS 1 equivalent
		if msg is not None: rclpy.logging.get_logger().info(msg)
		rate.sleep()

	return True

'''
	Callers for PLANSYS2 Services
'''
# Need to check if this is equivalent to ROS 1
def try_call_srv(topic, msg_ty=Empty):
	"""
    Tries to call a ROS service with a specified topic and message type.

    This function waits for the service to be available, then sends a request to the service and waits for the response. If the service call fails for any reason, an error message is logged.

    Parameters
    ----------
    topic : str
        The topic of the service to call.
    msg_ty : Type[rosidl_runtime_py.message.Message], optional
        The type of the service message. Default is std_srvs.srv.Empty.

    Returns
    -------
    bool
        True if the service call was successful and the response was not None, False otherwise.
    """
	rclpy.wait_for_service(topic)
	try:
		query_proxy = rclpy.create_client(msg_ty, topic)
		req = msg_ty.Request()
		future = query_proxy.call_async(req)
		rclpy.spin_until_future_complete(query_proxy, future)
		if future.result() is not None:
			return True
		else:
			rclpy.get_logger().error(f"MissionGoalManager Service call to {topic} failed")
			return False
	except Exception as e:
		rclpy.get_logger().error(f"MissionGoalManager Service call to {topic} failed: {e}")
		return False

def call_problem_generator(): return try_call_srv('/problem_expert/get_problem'                    , Empty) 
def call_plan_generator():	  return try_call_srv('/planner/get_plan'			                        , Empty)
##def call_parser():			  return try_call_srv('/rosplan_parsing_interface/parse_plan'				, Empty) #####
def call_mission_planning():  return try_call_srv('/harpia/mission_planning'							, Empty)

def call_dispatch(node):
	"""
	Calls the execute plan action server to dispatch a plan.

	This function creates an action client for the execute plan action server, sends an empty goal to the server, and waits for the result.

	Parameters
	----------
	node : rclpy.node.Node
		The ROS node to use for creating the action client and spinning.

	Returns
	-------
	bool
		True if the action server returned a result, False otherwise.
	"""
	action_client = ActionClient(node, plansys2_msgs.action.ExecutePlan, '/executor/execute_plan')
	goal_msg = plansys2_msgs.action.ExecutePlan.Goal()
	action_client.wait_for_server()
	future = action_client.send_goal_async(goal_msg)
	rclpy.spin_until_future_complete(node, future)
	return future.result() is not None

def cancel_dispatch(node, action_client):
    """
    Cancels the execution of a plan if it is currently being dispatched.

    This function checks if the execute plan action server is currently active. If it is, it sends a cancel request to the server and returns True. If it is not, it logs a message and returns False.

    Parameters
    ----------
    node : rclpy.node.Node
        The ROS node to use for logging.
    action_client : rclpy.action.ActionClient
        The action client for the execute plan action server.

    Returns
    -------
    bool
        True if a cancel request was sent, False otherwise.
    """
    if action_client.is_active():
        node.get_logger().info("Cancelling dispatch...")
        action_client.cancel_goal()
        return True
    else:
        node.get_logger().info("No active plan to cancel.")
        return False

######

'''
	Fuctions to calc the regios distances
'''
def calc_distances(regions):
	"""
    Calculates the Euclidean distances between all pairs of regions.

    This function iterates over all pairs of distinct regions and calculates the Euclidean distance between their centers. It then creates a function for each pair that returns this distance when called with the names of the two regions as arguments.

    Parameters
    ----------
    regions : list
        A list of region objects. Each region object should have a 'name' attribute and a 'center' attribute, where 'center' is an object with a 'cartesian' attribute that is a tuple of coordinates.

    Returns
    -------
    list
        A list of function objects. Each function object takes two arguments, both of which are key-value pairs where the key is "region" and the value is the name of a region, and returns the Euclidean distance between the centers of the two regions.
    """
	out = []
	for x in regions:
		for y in regions:
			if x == y: continue

			out.append(
				create_function(
					"distance",
					euclidean_distance(x.center.cartesian, y.center.cartesian),
					[KeyValue("region", x.name), KeyValue("region", y.name)]
				)
			)

	return out

def euclidean_distance(a, b):
	"""
    Calculates the Euclidean distance between two points.

    This function uses the Pythagorean theorem to calculate the Euclidean distance between two points in a 2D space.

    Parameters
    ----------
    a : object
        The first point. This object should have 'x' and 'y' attributes representing its coordinates.
    b : object
        The second point. This object should also have 'x' and 'y' attributes representing its coordinates.

    Returns
    -------
    float
        The Euclidean distance between the two points.
    """
	return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

def geo_to_cart(geo_point, geo_home):
	"""
    Converts geographic coordinates to Cartesian coordinates.

    This function takes a geographic point and a reference geographic point (home), and converts the geographic point's latitude, longitude, and altitude into Cartesian x, y, and z coordinates relative to the home point.

    Parameters
    ----------
    geo_point : object
        The geographic point to convert. This object should have 'latitude', 'longitude', and 'altitude' attributes representing its geographic coordinates.
    geo_home : object
        The reference geographic point (home). This object should also have 'latitude' and 'longitude' attributes representing its geographic coordinates.

    Returns
    -------
    Point
        The Cartesian coordinates of the geographic point relative to the home point. This is an object with 'x', 'y', and 'z' attributes.
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

'''
	Functions to manipulate Knowledge base
'''

def try_update_knowledge(item, update_type, service):
	"""
    Tries to update the knowledge base using a specified service.

    This function waits for the service to be available, then sends a request to the service to update the knowledge base. If the service call fails for any reason, an error message is logged.

    Parameters
    ----------
    item : str
        The item to update in the knowledge base.
    update_type : int
        The type of update to perform. This should be one of the constants defined in the KnowledgeUpdateService message.
    service : str
        The name of the service to call.

    Returns
    -------
    bool
        True if the service call was successful, False otherwise.
    """
	rclpy.wait_for_service(service)
	try:
		query_proxy = rclpy.ServiceProxy(service, KnowledgeUpdateService)
		query_proxy(update_type, item)
		return True
	except rclpy.ServiceException as e:
		rclpy.logerr(f"!!!PLANSYS2: MissionGoalManager Service call to {topic} failed: {e}")
		return False

def call_clear():		   return try_call_srv('/problem_expert/clear_problem_predicate', Empty)
# Substitui '/rosplan_knowledge_base/clear'

# Adicionei os serviços correspondentes no plansys2
def add_instance(item):    return try_update_knowledge(item, KB_UPDATE_ADD_KNOWLEDGE, '/problem_expert/add_problem_instance')
def remove_instance(item): return try_update_knowledge(item, KB_UPDATE_RM_KNOWLEDGE, '/problem_expert/remove_problem_instance')
def add_goal(item):		   return try_update_knowledge(item, KB_UPDATE_ADD_GOAL, '/problem_expert/add_problem_goal')
def remove_goal(item):	   return try_update_knowledge(item, KB_UPDATE_RM_GOAL, '/problem_expert/remove_problem_goal')
def add_metric(item):	   return try_update_knowledge(item, KB_UPDATE_ADD_METRIC, '/problem_expert/update_knowledge') # Não tem correspondente direto

def get_knowledge(name, topic):
	"""
    Retrieves knowledge about a specified item from a specified service.

    This function waits for the service to be available, then sends a request to the service to get knowledge about the specified item. If the service call fails for any reason, an error message is logged and an empty KnowledgeItem is returned.

    Parameters
    ----------
    name : str
        The name of the item to get knowledge about.
    topic : str
        The topic of the service to call.

    Returns
    -------
    KnowledgeItem
        The KnowledgeItem returned by the service, or an empty KnowledgeItem if the service call failed.
    """
	rclpy.wait_for_service(topic)
	try:
		query_proxy = rclpy.ServiceProxy(topic, GetAttributeService)
		return query_proxy(name)
	except rclpy.ServiceException as e:
		rclpy.logerr(f"MissionGoalManager Service call to {topic} failed: {e}")
		return KnowledgeItem()

def get_function(function_name):   return get_knowledge(function_name , '/problem_expert/get_problem_functions')
# Substitui '/rosplan_knowledge_base/state/functions'

def get_goal(goal_name):		   return get_knowledge(goal_name, '/problem_expert/get_problem_goal')
# Substitui '/rosplan_knowledge_base/state/goals'

def get_predicate(predicate_name): return get_knowledge(predicate_name, '/problem_expert/get_problem_predicate')
# Substitui  '/rosplan_knowledge_base/state/propositions'

def create_object(item_name, item_type):
	"""
    Creates a KnowledgeItem object with specified name and type.

    This function initializes a KnowledgeItem object, sets its knowledge_type to INSTANCE, and sets its instance_type and instance_name to the specified values.

    Parameters
    ----------
    item_name : str
        The name of the instance.
    item_type : str
        The type of the instance.

    Returns
    -------
    KnowledgeItem
        The created KnowledgeItem object.
    """
	instance = KnowledgeItem()
	instance.knowledge_type = KnowledgeItem.INSTANCE
	instance.instance_type = item_type
	instance.instance_name = item_name

	return instance

def create_function(attribute_name, function_value, values=[]):
	"""
    Creates a KnowledgeItem object representing a function with specified attribute name, function value, and values.

    This function initializes a KnowledgeItem object, sets its knowledge_type to FUNCTION, and sets its attribute_name, function_value, and values to the specified values.

    Parameters
    ----------
    attribute_name : str
        The name of the attribute.
    function_value : float
        The value of the function.
    values : list, optional
        The values of the function. Default is an empty list.

    Returns
    -------
    KnowledgeItem
        The created KnowledgeItem object.
    """
	instance = KnowledgeItem()

	instance.knowledge_type = KnowledgeItem.FUNCTION
	instance.attribute_name = attribute_name
	instance.values			= values
	instance.function_value = function_value

	return instance

def create_predicate(attribute_name, values=[], is_negative=False):
	"""
    Creates a KnowledgeItem object representing a predicate with specified attribute name, values, and negativity.

    This function initializes a KnowledgeItem object, sets its knowledge_type to FACT, and sets its attribute_name, values, and is_negative to the specified values.

    Parameters
    ----------
    attribute_name : str
        The name of the attribute.
    values : list, optional
        The values of the predicate. Default is an empty list.
    is_negative : bool, optional
        Whether the predicate is negative. Default is False.

    Returns
    -------
    KnowledgeItem
        The created KnowledgeItem object.
    """
	instance = KnowledgeItem()

	instance.knowledge_type = KnowledgeItem.FACT
	instance.attribute_name = attribute_name
	instance.values			= values
	instance.is_negative	= is_negative

	return instance

def create_metric(optimization, item):
	"""
    Creates a KnowledgeItem object representing an expression with specified optimization and item.

    This function initializes a KnowledgeItem object, sets its knowledge_type to EXPRESSION, and sets its optimization and expr.tokens to the specified values.

    Parameters
    ----------
    optimization : str
        The optimization of the expression.
    item : str
        The item of the expression.

    Returns
    -------
    KnowledgeItem
        The created KnowledgeItem object.
    """
	instance = KnowledgeItem()

	instance.knowledge_type = KnowledgeItem.EXPRESSION
	instance.optimization	= optimization
	instance.expr.tokens	= item

	return instance

def set_distances(map, goals, latitude, longitude):
	"""
    Sets the distances from the current location to all regions in the map.

    This function calculates the Euclidean distance from the current location to the center of each region in the map, and adds these distances to the knowledge base as function instances. The current location is specified by its latitude and longitude.

    Parameters
    ----------
    map : Map
        The map object, which contains the regions and bases.
    goals : list
        A list of goal objects. Each goal object should have a 'region' attribute.
    latitude : float
        The latitude of the current location.
    longitude : float
        The longitude of the current location.

    Returns
    -------
    None
    """
	#see distance to all regions
	geo_home = map.geo_home

	regions_name = []
	for g in goals:
		regions_name.append(g.region)

	#get current lat long
	cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), geo_home)

	distance = float('inf')
	closest_region = ''
	for base in map.bases:
		d = euclidean_distance(base.center.cartesian, cart_location)
		obj = create_function(
			"distance",
			round(d, 2),
			[KeyValue("region", "aux"), KeyValue("region", base.name)]
		)
		add_instance(obj)

	for region in map.roi:
		if(region.name in regions_name):
			d = euclidean_distance(cart_location, region.center.cartesian)
			obj = create_function(
				"distance",
				[KeyValue("region", "aux"), KeyValue("region", region.name)],
				round(d, 2)
			)
			add_instance(obj)

def regions_to_perform_action(goals, action):
	"""
    Returns the regions where a specified action needs to be performed.

    This function iterates over a list of goals, and for each goal where the action matches the specified action, it adds the region of the goal to the output list.

    Parameters
    ----------
    goals : list
        A list of goal objects. Each goal object should have a 'region' attribute and an 'action' attribute.
    action : str
        The action to match.

    Returns
    -------
    list
        A list of regions where the specified action needs to be performed.
    """
	return [step.region for step in goals if step.action == action]

def update_knowledge_base(uav, map, goals, at):
	"""
    Updates the knowledge base with information about the UAV, map, goals, and current location.

    This function first clears the knowledge base, then adds predicates, functions, and goals representing the current state of the UAV, the map, and the goals. The current location of the UAV is specified by the 'at' parameter.

    Parameters
    ----------
    uav : UAV
        The UAV object, which contains information about the UAV's capabilities.
    map : Map
        The map object, which contains the regions and bases.
    goals : list
        A list of goal objects. Each goal object should have a 'region' attribute and an 'action' attribute.
    at : str
        The name of the region where the UAV is currently located.

    Returns
    -------
    None
    """
	regions   = map.roi
	bases	   = map.bases
	pulverize = regions_to_perform_action(goals, 'pulverize')
	photo	  = regions_to_perform_action(goals, 'take_picture')
	end		  = regions_to_perform_action(goals, 'end')

	# In this version I'm only adding in the initial 
	# I had to make it as an for to prevent repetition
	goals_regions = set(pulverize + photo)
	regions_obj = [r for r in regions if r.name in goals_regions] + bases

	call_clear()

	# set drone init

	#adding objetcts to knowlege base
	add_instance(create_predicate("at", [KeyValue("region", at)]))
	add_instance(create_predicate("can-go"))
	add_instance(create_predicate("can-take-pic"))
	add_instance(create_function("battery-capacity", uav.battery.capacity))
	add_instance(create_function("velocity", uav.frame.efficient_velocity))
	add_instance(create_function("battery-amount", 100))
	add_instance(create_function("recharge-rate-battery", uav.battery.recharge_rate))
	add_instance(create_function("discharge-rate-battery", uav.battery.discharge_rate))
	add_instance(create_function("input-amount", 0))
	add_instance(create_function("missionLength", 0))
	add_instance(create_function("input-capacity", uav.input_capacity))

	for r in regions:
		add_instance(create_object(str(r.name), "region"))
		add_instance(create_predicate("its-not-base", [KeyValue("region", r.name)]))

	for b in bases:
		add_instance(create_object(str(b.name), "base"))

	for fact in calc_distances(regions_obj):
		add_instance(fact)

	if pulverize:
		add_instance(create_predicate("has-pulverize-goal"))

	if photo:
		add_instance(create_predicate("has-picture-goal"))

	total_goals = 0

	for i in pulverize:
		add_instance(create_predicate("pulverize-goal", [KeyValue("region", i)]))
		add_instance(create_function("pulverize-path-len", 314, [KeyValue("region", i)]))
		add_goal(create_predicate("pulverized", [KeyValue("region", i)]))
		total_goals += 1

	for i in photo:
		add_instance(create_predicate("picture-goal", [KeyValue("region", i)]))
		add_instance(create_function("picture-path-len", 1000, [KeyValue("region", i)]))
		add_goal(create_predicate("takenImage", [KeyValue("region", i)]))
		total_goals += 1

	for i in end:
		add_goal(create_predicate("at", [KeyValue("region", i)]))

	add_instance(create_function("total-goals", total_goals))
	add_instance(create_function("goals-achived", 0))
	# add_metric(create_metric("minimize (total-time)", []))
	add_metric(create_metric("minimize (missionLength)", []))

def find_at(map, goals, latitude, longitude):
	"""
    Finds a base or region within a 50m radius of the given latitude and longitude.

    This function first converts the given latitude and longitude to Cartesian coordinates. It then iterates over all bases in the map, and returns the first base whose center is within 50m of the current location. If no such base is found, it iterates over all regions of interest in the map, and returns the first region whose center is within 50m of the current location. If no such region is found, it returns None.

    Parameters
    ----------
    map : Map
        The map object, which contains the bases and regions of interest.
    goals : list
        A list of goal objects. Each goal object should have a 'region' attribute.
    latitude : float
        The latitude of the current location.
    longitude : float
        The longitude of the current location.

    Returns
    -------
    Base or Region or None
        The base or region found within 50m of the current location, or None if no such base or region is found.
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
	"""
    Finds a base or region within a 50m radius of the given latitude and longitude.

    This function first converts the given latitude and longitude to Cartesian coordinates. It then iterates over all bases in the map, and returns the first base whose center is within 50m of the current location. If no such base is found, it iterates over all regions of interest in the map, and returns the first region whose center is within 50m of the current location. If no such region is found, it returns None.

    Parameters
    ----------
    map : Map
        The map object, which contains the bases and regions of interest.
    goals : list
        A list of goal objects. Each goal object should have a 'region' attribute.
    latitude : float
        The latitude of the current location.
    longitude : float
        The longitude of the current location.

    Returns
    -------
    Base or Region or None
        The base or region found within 50m of the current location, or None if no such base or region is found.
    """
	cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

	return min(map.bases, key=lambda base: euclidean_distance(base.center.cartesian, cart_location))

def call_path_planning(r_from, r_to, map):
	"""
    Calls the path planning service to find a path from one region to another.

    This function waits for the path planning service to be available, then sends a request to the service to find a path from the center of the 'r_from' region to the center of the 'r_to' region. If the service call fails for any reason, an error message is logged and None is returned.

    Parameters
    ----------
    r_from : Region
        The region to start from.
    r_to : Region
        The region to go to.
    map : Map
        The map object, which contains the regions and bases.

    Returns
    -------
    PathPlanningResponse or None
        The response from the path planning service, or None if the service call failed.
    """
	rclpy.wait_for_service("/harpia/path_planning")
	try:
		query_proxy = rclpy.ServiceProxy("/harpia/path_planning", PathPlanning)
		resp = query_proxy(r_from.center, r_to.center, r_from.name, r_to.name, 3, map)
		return resp
	except rclpy.ServiceException as e:
		rclpy.logerr("Service call failed: %s" % e)
		return None

# Huge changes made, problably will break
def replan(mission, uav, base, goals, op):
	"""
    Replans the mission based on the current state of the UAV, the base, and the goals.

    This function first checks if the mission has any goals and if the UAV's position is known. If not, it returns immediately. Then, it checks if there are any goals to pulverize or take pictures of. If the base is not specified, it tries to find the current location of the UAV. If the base is specified, it sets the current location to the base. It then updates the knowledge base with the current location, the total number of goals, and the current battery amount. Finally, it updates the goals based on the operation specified by the 'op' parameter.

    Parameters
    ----------
    mission : Mission
        The mission object, which contains the map and the goals.
    uav : UAV
        The UAV object, which contains information about the UAV's capabilities and current state.
    base : Base or None
        The base object, which contains information about the base, or None if the base is not specified.
    goals : list or None
        A list of goal objects, or None if the goals are not specified.
    op : int
        The operation to perform. 0 means to update the current goals, 1 means to add new goals, and 2 means to remove goals.

    Returns
    -------
    None
    """
	rclpy.loginfo("CANCEL DISPATCH - MissionGoalManager")

	if not wait_until(lambda: mission.goals != [], msg="Waiting for Regions..."):
		return
	if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."):
		return

	if goals != None:
		pulverize = regions_to_perform_action(goals, 'pulverize')
		photo = regions_to_perform_action(goals, 'take_picture')

	if base == None:
		# Here we want to continue the mission.
		at_move = get_predicate("at-move")
		at = get_predicate("at")

		if len(at.attributes) == 0:
			if len(at_move.attributes) > 0:
				remove_instance(at_move.attributes[0])

			# codigo achar at
			base_or_region_nearby = find_at(mission.map, mission.goals, uav.latitude, uav.longitude)
			if base_or_region_nearby != None:
				# We are at a base or a region.
				rclpy.loginfo(f'at = {base_or_region_nearby}')
				at = create_predicate("at", [KeyValue("region", base_or_region_nearby.name)])
			else:
				# for updates on the mission while on the move, I created a new auxiliar region to start the plan
				# add new region to problem
				add_instance(create_object("aux", "region"))
				add_instance(create_predicate("its-not-base", [KeyValue("region", "aux")]))
				set_distances(mission.map, mission.goals, uav.latitude, uav.longitude)

				at = create_predicate("at", [KeyValue("region", "aux")])
		else:
			at = at.attributes[0]
	else:
		# We want to go home
		at_move = get_predicate("at-move")
		if len(at_move.attributes) > 0:
			remove_instance(at_move.attributes[0])

		at_kb = get_predicate("at")
		rclpy.loginfo(at_kb)
		if len(at_kb.attributes) > 0:
			remove_instance(at_kb.attributes[0])

		rclpy.loginfo(f"base = {base}")
		at = create_predicate("at", [KeyValue("base", base.name)])

	add_instance(at)

	# saving the current total goals to manage goals
	f = get_function("total-goals")
	total_goals = f.attributes[0].function_value
	remove_instance(f.attributes[0])

	bat = get_function("battery-amount")
	remove_instance(bat.attributes[0])

	if not wait_until(lambda: uav.battery is not None, msg="Waiting for UAV battery..."):
		return

	obj = create_function("battery-amount", uav.battery)
	add_instance(obj)
	# has_end is a variable to verify if a mission has already a designed end
	has_end = False
	if op == 0:
		# getting current goals and verifying if it already done
		for goal in get_goal('').attributes:
			if goal.attribute_name != "at":
				for goal_achieved in get_predicate(goal.attribute_name).attributes:
					if goal.values[0].value == goal_achieved.values[0].value:
						f.attributes[0].function_value = f.attributes[0].function_value - 1
						remove_goal(goal)
			else:
				has_end = True
		add_instance(create_function("total-goals", total_goals))

	elif op == 1:
		if pulverize:
			add_instance(create_predicate("has-pulverize-goal"))

		if photo:
			add_instance(create_predicate("has-picture-goal"))

		for i in pulverize:
			add_instance(create_predicate("pulverize-goal", [KeyValue("region", i)]))
			add_instance(create_function("pulverize-path-len", 314, [KeyValue("region", i)]))
			add_goal(create_predicate("pulverized", [KeyValue("region", i)]))
			total_goals = total_goals + 1

		for i in photo:
			add_instance(create_predicate("picture-goal", [KeyValue("region", i)]))
			add_instance(create_function("picture-path-len", 1000, [KeyValue("region", i)]))
			add_goal(create_predicate("taken-image", [KeyValue("region", i)]))
			total_goals = total_goals + 1

	elif op == 2:
		for goal in get_goal('').attributes:
			if goal.attribute_name != "at":
				for goal_to_remove in goals:
					if goal.values[0].value == goal_to_remove.values[0].value:
						f.attributes[0].function_value = f.attributes[0].function_value - 1
						remove_goal(goal)
			else:
				has_end = True

		add_instance(create_function("total-goals", total_goals))

		add_instance(create_function("total-goals", total_goals))
		print(obj)
		add_instance(obj)

		for g in goals:
			mission.goals.append(g)
		regions = mission.map.roi
		pulverize = regions_to_perform_action(mission.goals, 'pulverize')
		photo = regions_to_perform_action(mission.goals, 'take_picture')

		goals_regions = set(pulverize + photo)
		bases = mission.map.bases
		regions_obj = [r for r in regions if r.name in goals_regions] + bases

		for fact in calc_distances(regions_obj):
			add_instance(fact)

def replan1(mission, goals, op, uav):
	"""
    Replans the mission based on the current state of the UAV and the goals.

    This function first logs the cancellation of the dispatch. It then determines the regions where the 'pulverize' and 'take_picture' actions need to be performed. It waits for the UAV's position and the mission goals to be available. If the UAV is not at a base or a region, it creates an auxiliary region to start the plan. It then updates the total number of goals and the battery amount. Finally, it updates the goals based on the operation specified by the 'op' parameter.

    Parameters
    ----------
    mission : Mission
        The mission object, which contains the map and the goals.
    goals : list
        A list of goal objects. Each goal object should have a 'region' attribute and an 'action' attribute.
    op : int
        The operation to perform. 0 means to update the current goals, 1 means to add new goals, and 2 means to remove goals.
    uav : UAV
        The UAV object, which contains information about the UAV's capabilities and current state.

    Returns
    -------
    None
    """
	rclpy.loginfo("CANCEL DISPATCH - MissionGoalManager")

	pulverize = regions_to_perform_action(goals, 'pulverize')
	photo	  = regions_to_perform_action(goals, 'take_picture')

	if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."): return
	if not wait_until(lambda: mission.goals != []	  , msg="Waiting for Regions..."): return
	# print(mission.goals)
	
	# Here we want to continue the mission.
	at_move = get_predicate("at-move")
	at = get_predicate("at")

	# rospy.loginfo(f"at={at}")
	# rospy.loginfo(f"len at_={len(at.attributes)}")

	if len(at.attributes) == 0:
		if len(at_move.attributes) > 0:
			remove_instance(at_move.attributes[0])

		#codigo achar at
		base_or_region_nearby = find_at(mission.map, mission.goals, uav.latitude, uav.longitude)
		if base_or_region_nearby != None:
			# We are at a base or a region.
			# rospy.loginfo(f'at = {base_or_region_nearby}')
			at = create_predicate("at", [KeyValue("region", base_or_region_nearby.name)])
		else:
			# for updates on the mission while on the move, I created a new auxiliar region to start the plan
			# add new region to problem
			add_instance(create_object("aux", "region"))
			add_instance(create_predicate("its-not-base", [KeyValue("region", "aux")]))
			set_distances(mission.map, mission.goals, uav.latitude, uav.longitude)

			at = create_predicate("at", [KeyValue("region", "aux")])
	else:
		at = at.attributes[0]
	
	add_instance(at)

	# saving the current total goals to manage goals
	f = get_function("total-goals")
	total_goals = f.attributes[0].function_value
	print(f.attributes[0])
	#.attributes[0]
	remove_instance(f.attributes[0])

	# has_end is a variable to verify if a mission has already a designed end
	has_end = False

	bat = get_function("battery-amount")
	remove_instance(bat.attributes[0])

	if not wait_until(lambda: uav.battery is not None, msg="Waiting for UAV battery..."): return

	obj = create_function("battery-amount", uav.battery)
	add_instance(obj)

	if (op == 1):
		if pulverize:
			add_instance(create_predicate("has-pulverize-goal"))

		if photo:
			add_instance(create_predicate("has-picture-goal"))

		for i in pulverize:
			add_instance(create_predicate("pulverize-goal", [KeyValue("region", i)]))
			add_instance(create_function("pulverize-path-len", 314, [KeyValue("region", i)]))
			add_goal(create_predicate("pulverized", [KeyValue("region", i)]))
			total_goals = total_goals + 1

		for i in photo:
			add_instance(create_predicate("picture-goal", [KeyValue("region", i)]))
			add_instance(create_function("picture-path-len", 1000, [KeyValue("region", i)]))
			add_goal(create_predicate("taken-image", [KeyValue("region", i)]))
			total_goals = total_goals + 1

		add_instance(create_function("total-goals", total_goals))
		print(obj)
		add_instance(obj)

		for g in goals: mission.goals.append(g)
		regions   = mission.map.roi
		pulverize = regions_to_perform_action(mission.goals, 'pulverize')
		photo	  = regions_to_perform_action(mission.goals, 'take_picture')

		goals_regions = set(pulverize + photo)
		bases	   = mission.map.bases
		regions_obj = [r for r in regions if r.name in goals_regions] + bases

		for fact in calc_distances(regions_obj):
			add_instance(fact)

def go_to_base(mission, uav):
	"""
    Directs the UAV to go to the nearest base and land.

    This function first waits for the UAV's position to be available. If the position is not available, it logs an error message and returns. It then finds the current location of the UAV and the nearest base. It calls the path planning service to find a route from the current location to the base, and sends this route to the UAV. It sets the UAV's mode to 'AUTO.MISSION', and waits for the UAV to arrive at the base. Finally, it lands the UAV and waits for 30 seconds.

    Parameters
    ----------
    mission : Mission
        The mission object, which contains the map and the goals.
    uav : UAV
        The UAV object, which contains information about the UAV's capabilities and current state.

    Returns
    -------
    Base
        The base where the UAV landed.
    """

	rate = rclpy.Rate(1)

	if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."):
		rclpy.logerr("CRITICAL - MissionGoalManager was terminated while going to nearest base and will not finish the action")
		return

	at = find_at(mission.map, mission.goals, uav.latitude, uav.longitude)
	base = find_nearest_base(mission.map, uav.latitude, uav.longitude)

	route = call_path_planning(at, base, mission.map)

	# send route to uav
	clear_mission()
	rclpy.loginfo("Send Route")
	send_route(route.waypoints)

	# set mode to mission
	rclpy.loginfo("Set Mode")
	set_mode("AUTO.MISSION")

	# wait to arrive.
	uav.current = 0
	while uav.current < len(route.waypoints.waypoints):
		rclpy.sleep(1)

	# land
	land()
	rclpy.sleep(30)

	return base

'''
	Callers for MAVRos Services
'''

def mavros_cmd(topic, msg_ty, error_msg="MAVROS command failed: ", **kwargs):
	"""
    Sends a command to MAVROS via a specified service.

    This function waits for the specified service to be available, then sends a request to the service using the provided message type and arguments. If the service call is successful, it logs the response. If the service call fails for any reason, it logs an error message.

    Parameters
    ----------
    topic : str
        The name of the service to which the command should be sent.
    msg_ty : Type
        The type of the message to be sent.
    error_msg : str, optional
        The error message to be logged if the service call fails. Default is "MAVROS command failed: ".
    **kwargs : dict
        Additional keyword arguments to be passed to the service call.

    Returns
    -------
    None
    """
	rclpy.wait_for_service(topic)
	try:
		service_proxy = rclpy.ServiceProxy(topic, msg_ty)
		response = service_proxy(**kwargs)
		rclpy.loginfo(response)
	except rclpy.ServiceException as e:
		rclpy.logerr(f"{error_msg} {e}")

def land():
	"""
    Sends a command to MAVROS to land the UAV.

    This function sends a command to the '/mavros/cmd/land' service to land the UAV. The command includes the altitude, latitude, longitude, minimum pitch, and yaw. If the service call fails for any reason, it logs an error message.

    Parameters
    ----------
    None

    Returns
    -------
    None
    """
	mavros_cmd(
		'/mavros/cmd/land',
		CommandTOL,
		error_msg="Landing failed",
		altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
	)

def set_mode(mode):
	"""
    Sets the mode of the UAV via MAVROS.

    This function sends a command to the '/mavros/set_mode' service to set the mode of the UAV. The command includes the mode to be set. If the service call fails for any reason, it logs an error message.

    Parameters
    ----------
    mode : str
        The mode to be set. This should be one of the modes recognized by MAVROS.

    Returns
    -------
    None
    """
	mavros_cmd(
		'/mavros/set_mode',
		SetMode,
		error_msg="Set mode failed",
		custom_mode=mode
	)

def clear_mission():
	"""
    Clears the current mission via MAVROS.

    This function sends a command to the '/mavros/mission/clear' service to clear the current mission. If the service call fails for any reason, it logs an error message.

    Parameters
    ----------
    None

    Returns
    -------
    None
    """
	mavros_cmd(
		'/mavros/mission/clear',
		WaypointClear,
		error_msg="Clear mission failed"
	)

def send_route(route):
	"""
    Sends a route to the UAV via MAVROS.

    This function sends a command to the '/mavros/mission/push' service to send a route to the UAV. The command includes the start index and the waypoints of the route. If the service call fails for any reason, it logs an error message.

    Parameters
    ----------
    route : Route
        The route to be sent. This should be an object that has a 'current_seq' attribute and a 'waypoints' attribute.

    Returns
    -------
    None
    """
	mavros_cmd(
		'/mavros/mission/push',
		WaypointPush,
		error_msg="Send route failed",
		start_index=route.current_seq,
		waypoints=route.waypoints
	)

if __name__ == "__main__":
	server = ActionServer()
	server.run()
