#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.exceptions import ROSInterruptException

import sys, select

import math
import json
import time
import os
import argparse
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

from std_msgs.msg import String
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from interfaces.msg import *
from interfaces.srv import *


feedback = 0

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self.feedback = 0
        self.client = self.create_client(WeatherCheck, '/harpia/weather_check')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = WeatherCheck.Request()

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.status
        self.get_logger().info('Received feedback: {}'.format(self.feedback))

    def send_request(self):
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    test_client = TestClient()
    test_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(test_client)
        if test_client.future.done():
            try:
                response = test_client.future.result()
            except Exception as e:
                test_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                test_client.get_logger().info(
                    'Result: %s' % (response.replan))
            break

    test_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()