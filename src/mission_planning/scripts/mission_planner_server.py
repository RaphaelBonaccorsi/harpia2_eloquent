#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile

import sys, select
import math
import json
import time
import os
import psutil
from shutil import copyfile
from itertools import count
from std_srvs.srv import Empty
"""
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *
"""

from std_msgs.msg import String

from interfaces.srv import *
from interfaces.msg import *

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

def parse_file_plan():
    """
    Parses the plan file and logs the plan.

    This function reads the plan.pddl file located in the 'pddl' directory of the Harpia project's root directory. It checks if a solution was found, calculates the CPU time and total time, and logs the plan to a new file in the 'results/plans' directory. The new file is named with an integer id, which is the smallest non-negative integer that is not already used as a file name in the directory.

    Returns
    -------
    tuple
        A tuple containing four elements:
        - success (bool): True if a solution was found, False otherwise.
        - cpu_time (float): The CPU time used to find the solution, or infinity if no solution was found.
        - total_time (float): The total time of the plan.
        - id (int): The id used as the name of the log file.
    """
    root_dir = get_harpia_root_dir()
    plan_path = os.path.join(root_dir, 'pddl/plan.pddl')

    success = False
    cpu_time = float('Inf')
    total_time = 0

    for line in open(plan_path):
        if 'Solution Found' in line: success = True
        if '; Time' in line:
            aux = line.split(' ')
            cpu_time = float(aux[-1].rstrip())
        if ': (' in line:
            aux = line.split(' ')
            # TODO: using `eval` is never very safe, change this to something more robust
            total_time += eval(aux[-1].rstrip())[0]

    # log path
    log_dir = os.path.join(root_dir, "results/plans/")

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    ith_log_name = lambda i: os.path.join(log_dir, f"{i}.pddl")

    # Create an iterator over the unused log file names and use the next one available
    log_names = ((ith_log_name(i), i) for i in count(0) if not os.path.exists(ith_log_name(i)))
    log_path, id = next(log_names)

    copyfile(plan_path, log_path)
   
    return success, cpu_time, total_time, id

def log():
    """
    Logs the plan details to a JSON file.

    This function reads the mission log from the 'results/mission_log.json' file in the Harpia project's root directory. It parses the plan file to get the success status, CPU time, total time, and plan id. It then appends the CPU time and plan id to the last entry in the mission log and writes the updated log back to the file.

    """
    # log path
    root_dir = get_harpia_root_dir()
    log_path = os.path.join(root_dir, "results/mission_log.json")

    # open log
    with open(log_path, "r") as log_file:
        log_file = json.load(log_file)


    sucsses, cpu_time, total_time, plan_id = parse_file_plan()
    # add replan
    log_file[-1]['cpu_time'].append(cpu_time) 
    log_file[-1]['plans'].append(plan_id) 

    # dump log
    with open(log_path, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)

class Plan(Node):
    """
    A ROS2 node that subscribes to the complete plan and action dispatch topics.

    This node subscribes to the 'rosplan_parsing_interface/complete_plan' and '/rosplan_plan_dispatcher/action_dispatch' topics. It stores the complete plan and the current action, and provides methods to check if the mission has ended and to unsubscribe from the complete plan topic.

    Attributes
    ----------
    sub : Subscription
        The subscription to the 'rosplan_parsing_interface/complete_plan' topic.
    sub2 : Subscription
        The subscription to the '/rosplan_plan_dispatcher/action_dispatch' topic.
    plan : CompletePlan
        The complete plan received from the 'rosplan_parsing_interface/complete_plan' topic.
    current_action : ActionDispatch
        The current action received from the '/rosplan_plan_dispatcher/action_dispatch' topic.
    """
    def __init__(self):
        """
        Initializes the Plan node and creates the subscriptions.
        """
        super().__init__('plan')
        self.sub = self.create_subscription(CompletePlan, "rosplan_parsing_interface/complete_plan", self.plan_callback, 10)
        self.sub2 = self.create_subscription(ActionDispatch, "/rosplan_plan_dispatcher/action_dispatch", self.action_callback, 10)
        self.plan = CompletePlan()
        self.current_action = ActionDispatch()

    def plan_callback(self, data):
        """
        Callback function for the 'rosplan_parsing_interface/complete_plan' subscription.

        This function is called when a new message is published on the 'rosplan_parsing_interface/complete_plan' topic. It stores the received plan and destroys the subscription.

        Parameters
        ----------
        data : CompletePlan
            The complete plan received from the topic.
        """
        self.plan = data
        self.sub.destroy()

    def action_callback(self, data):
        """
        Callback function for the '/rosplan_plan_dispatcher/action_dispatch' subscription.

        This function is called when a new message is published on the '/rosplan_plan_dispatcher/action_dispatch' topic. It stores the received action.

        Parameters
        ----------
        data : ActionDispatch
            The current action received from the topic.
        """
        self.current_action = data

    def end_mission(self):
        """
        Checks if the mission has ended.

        This function checks if the id of the last action in the plan is equal to the id of the current action. If they are equal, it means that the mission has ended.

        Returns
        -------
        bool
            True if the mission has ended, False otherwise.
        """
        print(self.plan.plan[-1].action_id)
        print(self.current_action.action_id)
        return self.plan.plan[-1].action_id == self.current_action.action_id

    def unsubscribe(self):
        """
        Unsubscribes from the 'rosplan_parsing_interface/complete_plan' topic.

        This function destroys the subscription to the 'rosplan_parsing_interface/complete_plan' topic.
        """
        self.sub.destroy()

def mission_planning(self, req):
    """
    Executes the mission planning process.

    This function initializes a Plan object and checks if the mission has ended. If the mission has not ended, it generates a problem, calls the plan generator, logs the plan, calls the parser, and finally calls the dispatcher. If any of these steps fail, it returns None. If all steps succeed, it returns True.

    Parameters
    ----------
    req : Request
        The request object containing the initial action id, and the drone's battery information including the discharge and recharge rates.

    Returns
    -------
    bool or None
        True if the mission planning process was successful, None if any step failed.
    """
    self.plan = Plan()
    self.has_plan = False
    if self.has_plan and self.plan.end_mission():
        return True

    self.get_logger().info("Creating problem")
    if not self.call_problem_generator():
        return None

    rclpy.sleep(0.1)  # Sleeps for 0.1 sec

    self.get_logger().info("Calling Plan generator")
    if not self.call_plan_generator():
        return None

    self.has_plan = True

    self.get_logger().info("Calling Parser")
    self.log()
    if not self.call_parser():
        return None

    self.get_logger().info("Call Dispatch")
    self.get_logger().info(str(req))

    if not self.call_dispatch(): 
        return None

    return True
'''
    Callers for ROSPlan Services
'''
def try_call_srv(node, topic, srv_type):
    """
    Attempts to call a ROS service.

    This function waits for a ROS service to become available and then attempts to call it. If the service call succeeds, it returns True. If the service call fails, it logs the error and returns None.

    Parameters
    ----------
    node : rclpy.node.Node
        The ROS node to use for the service call.
    topic : str
        The name of the ROS service to call.
    srv_type : Type[rclpy.service.Service]
        The type of the ROS service.

    Returns
    -------
    bool or None
        True if the service call was successful, None if the service call failed.
    """
    cli = node.create_client(srv_type, topic)

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    req = srv_type.Request()

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        return True
    else:
        node.get_logger().error('Service call failed %r' % (future.exception(),))
        return None

def call_problem_generator(): return try_call_srv('/rosplan_problem_interface/problem_generation_server', Empty)
def call_plan_generator():    return try_call_srv('/rosplan_planner_interface/planning_server', Empty)
def call_parser():            return try_call_srv('/rosplan_parsing_interface/parse_plan', Empty)
def call_dispach():            return try_call_srv('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)

# def call_dispach():
#     rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
#     try:
#         query_proxy = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
#         result = query_proxy()
#         return result.success
#     except rospy.ServiceException as e:
#         rospy.logerr(f"Service call failed: {e}")
#         # Should we return false here? I guess...
#         return None

def mission_planning_server():
    """
    Initializes a server for the mission planning service.

    This function initializes a ROS node named 'mission_planning_server' and creates a service named 'harpia/mission_planning' of type Empty. The service uses the mission_planning function to handle requests. After the service is created, the function logs a message indicating that the service is ready and then spins the node to keep it running.

    """
    rospy.init_node('mission_planning_server')
    srv = rospy.Service('harpia/mission_planning', Empty, mission_planning)
    rospy.loginfo("Mission Planning Ready")
    srv.spin()

if __name__ == '__main__':
    mission_planning_server()
