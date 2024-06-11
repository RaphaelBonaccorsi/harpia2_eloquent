#!/usr/bin/env python3

import sys, traceback

import math
import os
import pandas as pd
import pickle
import json
import rclpy
import time
import requests

from libs import predictor
"""
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *
"""
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from sensor_msgs.msg import BatteryState

from interfaces.msg import *
from mavros_msgs.msg import *
from interfaces.srv import *

from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException

# ---
# CLASSES

class Plan(Node):
    def __init__(self):
        super().__init__('plan')
        self.sub  = self.create_subscription(CompletePlan, "rosplan_parsing_interface/complete_plan", self.plan_callback, 10)   
        self.plan = CompletePlan()
    
    def plan_callback(self, data): 
        self.plan = data
        self.unsubscribe()

    def unsubscribe(self):
        self.sub.destroy()

class Drone(Node):
    def __init__(self):
        super().__init__('drone')
        self.sub = self.create_subscription(NavSatFix, 'mavros/global_position/global',  self.global_position_callback, 10)  
        self.sub1 = self.create_subscription(BatteryState, 'mavros/battery',  self.battery_state_callback, 10)  
        self.latitude = float("inf")
        self.longitude = float("inf")
        self.battery =  float("inf")
    
    def global_position_callback(self, data): 
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.unsubscribe()

    def battery_state_callback(self, data): 
        self.battery = data.percentage * 100
        # self.unsubscribe()

    def unsubscribe(self):
        self.sub.destroy()

# ---
# UTILS
def log(index):
    harpia_root_dir = get_harpia_root_dir()

    # log path
    log_path = os.path.join(harpia_root_dir, "results")
    log_json = os.path.join(log_path, "mission_log.json")

    # open log
    with open(log_json, "r") as log_file:
        log_file = json.load(log_file)

    # add replan
    log_file[-1]['knn'][index] += 1

    # dump log
    with open(log_json, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)

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

def wait_until(check, msg=None, rate=1):
    rate = rclpy.Rate(rate)

    while not check():
        if rclpy.is_shutdown(): return False
        if msg is not None: self.get_logger().info(msg)
        rate.sleep()

    return True


# ---
# SERVER

class WeatherCheckServer(Node):
    def __init__(self):
        super().__init__('weather_check_server')
        self.srv = self.create_service(WeatherCheck, 'harpia/weather_check', self.weather_check)

    def weather_check(self, request, response):
        p = Plan()
        uav = Drone()

        while uav.latitude ==  float("inf"):
                print("Waiting for UAV info...")
                time.sleep(1)

        apikey = '' # api key for the openwheater api 

        address="https://api.openweathermap.org/data/2.5/weather?lat={}&lon={}&appid={}&units=metric".format(uav.latitude,uav.longitude,apikey)

        conect =  requests.get(address)

        current_weather = conect.json()

        if (current_weather['cod'] == 200):
            print('we have weather info')
            print(current_weather['weather'])
        else:
            print('error getting weather info')

        response.replan = 0
        return response

def main(args=None):
    rclpy.init(args=args)

    weather_check_server = WeatherCheckServer()

    rclpy.spin(weather_check_server)

    weather_check_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()