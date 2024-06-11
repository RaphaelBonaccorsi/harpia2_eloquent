#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from mavros_msgs.msg import CommandTOL
from nav_msgs.msg import Odometry
from interfaces.msg import DronePose, UAV as UAVHarpia
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, Imu, BatteryState

# Import custom libraries
from libs.anomalyDetection import checkAnomaly, pca_model, scaler_model, tree_model
from libs import NoiseGenerator, action

import time
import math
import numpy as np
import sys
import select

def sequential_noise(data):
    """
    Generate sequential noise data for various parameters.

    Parameters
    ----------
    data : dict
        The original data.

    Returns
    -------
    dict
        A dictionary containing noisy data for each parameter.
    """
    sequential = {
        'roll': [],
        'pitch': [],
        'yaw': [],
        'heading': [],
        'rollRate': [],
        'pitchRate': [],
        'yawRate': [],
        'groundSpeed': [],
        'climbRate': 0,  # ?
        'altitudeRelative': [],
        'throttlePct': []
    }
    for key in sequential:
        sequential[key] = NoiseGenerator.noisyData(data, key, 1., 5.)
    return sequential

class UAV(Node):
    """
    UAV class representing the UAV node.

    Attributes
    ----------
    sequential : dict
        Dictionary to store sequential UAV data.
    user_response_time : int
        User response time.
    classifier_win_time : int
        Classifier window time.
    action_win_time : int
        Action window time.
    sub_pose : Subscription
        Subscription to the drone pose topic.
    sub_hardware : Subscription
        Subscription to the UAV hardware topic.
    """
    def __init__(self):
        """
        Initialize the UAV node.
        """
        super().__init__('uav_node')
        self.sequential = {
            'roll': -math.inf,
            'pitch': -math.inf,
            'yaw': -math.inf,
            'heading': -math.inf,
            'rollRate': -math.inf,
            'pitchRate': -math.inf,
            'yawRate': -math.inf,
            'groundSpeed': -math.inf,
            'climbRate': -math.inf,
            'altitudeRelative': -math.inf,
            'throttlePct': -math.inf
        }
        self.user_response_time = 30
        self.classifier_win_time = 10
        self.action_win_time = 60
        self.sub_pose = self.create_subscription(
            DronePose,
            '/drone_info/pose',
            self.pose_callback,
            QoSProfile(depth=10)
        )
        self.sub_hardware = self.create_subscription(
            UAVHarpia,
            '/hapia/uav',
            self.hardware_callback,
            QoSProfile(depth=10)
        )

    def pose_callback(self, msg):
        """
        Callback function for pose subscription.

        Parameters
        ----------
        msg : DronePose
            The message containing the drone pose data.
        """
        self.sequential['roll'] = msg.roll
        self.sequential['pitch'] = msg.pitch
        self.sequential['yaw'] = msg.yaw
        self.sequential['heading'] = msg.heading
        self.sequential['rollRate'] = msg.roll_rate
        self.sequential['pitchRate'] = msg.pitch_rate
        self.sequential['yawRate'] = msg.yaw_rate
        self.sequential['groundSpeed'] = msg.ground_speed
        self.sequential['throttlePct'] = msg.throttle
        self.sequential['altitudeRelative'] = msg.alt_relative
        self.sequential['climbRate'] = msg.climb_rate

    def hardware_callback(self, msg):
        """
        Callback function for hardware subscription.

        Parameters
        ----------
        msg : UAVHarpia
            The message containing the UAV hardware data.
        """
        self.user_response_time = msg.fault_settings.user_response
        self.classifier_win_time = msg.fault_settings.classifier_time
        self.action_win_time = msg.fault_settings.action_time

def mavros_cmd(node, topic, msg_ty, error_msg="MAVROS command failed", **kwargs):
    """
    Send a command to a MAVROS service.

    Parameters
    ----------
    node : Node
        The ROS node.
    topic : str
        The topic name for the service.
    msg_ty : type
        The message type for the service.
    error_msg : str, optional
        The error message to display if the service call fails (default is "MAVROS command failed").
    **kwargs : dict
        Additional keyword arguments for the service request.
    """
    try:
        service_client = node.create_client(msg_ty, topic)
        while not service_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"Waiting for service {topic}...")
        request = msg_ty.Request()
        for key, value in kwargs.items():
            setattr(request, key, value)
        future = service_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            response = future.result()
            node.get_logger().info(str(response))
        else:
            node.get_logger().error(f"{error_msg} Service call failed.")
    except Exception as e:
        node.get_logger().error(f"{error_msg} {e}")

def land(node):
    """
    Command the UAV to land.

    Parameters
    ----------
    node : Node
        The ROS node.
    """
    mavros_cmd(
        node,
        '/mavros/cmd/land',
        CommandTOL,
        error_msg="Landing failed",
        altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
    )

def listener():
    """
    Main listener function to initialize the UAV node and handle anomaly detection.
    """
    rclpy.init()
    node = UAV()

    start_main = time.time()
    uav_stat = {
        'normal': 0,
        'noise': 0,
        'mild': 0,
        'abnormal': 0,
        'data': [],
        'last_state': 'none'
    }

    uav_action = []
    pct_normal = 0
    pct_noise = 0
    pct_mild = 0
    pct_abnormal = 0

    flag_error = False
    time_win = node.classifier_win_time

    start = time.time()
    module1 = pca_model()
    end = time.time()
    node.get_logger().info(f"Loaded PCA Module in {round((end-start), 3)} seconds.")

    start = time.time()
    module2 = scaler_model()
    end = time.time()
    node.get_logger().info(f"Loaded Scaler Module in {round((end-start), 3)} seconds.")

    start = time.time()
    module3 = tree_model()
    end = time.time()
    node.get_logger().info(f"Loaded Tree Module in {round((end-start), 3)} seconds.")

    node.get_logger().info("Anomaly Detector Ready")

    start_main = time.time()
    win = 0
    last_win = 0
    uav_stat, flag = checkAnomaly(node.sequential, uav_stat, module1, module2, module3, win)
    response_time = time.time()
    while flag:
        win = int(round((time.time() - start_main), 3) // time_win)
        if last_win != win:
            total = uav_stat['normal'] + uav_stat['noise'] + uav_stat['mild'] + uav_stat['abnormal']
            pct_normal = uav_stat['normal'] / total
            pct_noise = uav_stat['noise'] / total
            pct_mild = uav_stat['mild'] / total
            pct_abnormal = uav_stat['abnormal'] / total

            if (pct_noise <= 0.7 and pct_abnormal == 0.0 and pct_mild <= 0.05):
                uav_action.append(0)  # ok
            elif (pct_noise > 0.7 and pct_abnormal == 0.0 and pct_mild <= 0.1):
                uav_action.append(1)  # soft warning
            elif (0.1 < pct_mild <= 0.4):
                uav_action.append(2)  # warning
            elif (pct_abnormal < 0.05 and 0.4 < pct_mild <= 0.6):
                uav_action.append(4)  # base
            elif (pct_abnormal < 0.1 or pct_mild > 0.6):
                uav_action.append(8)  # base
            elif (pct_abnormal < 0.2):
                uav_action.append(16)  # base
            else:
                uav_action.append(32)  # land

            # Reset parameters
            last_win = win
            uav_stat = {key: 0 for key in uav_stat}
            uav_stat['data'] = []

        if flag_error and (10 < time.time() - start_main < 120):
            node.get_logger().info('noisyData')
            error_data = sequential_noise(node.sequential)
            uav_stat, flag = checkAnomaly(error_data, uav_stat, module1, module2, module3, win)
        else:
            uav_stat, flag = checkAnomaly(node.sequential, uav_stat, module1, module2, module3, win)

        action_win = int(node.action_win_time / node.classifier_win_time)
        flag_action = np.sum(uav_action[-action_win:])

        if flag_action > 24 and time.time() > response_time:
            node.get_logger().info("Do you want to continue? (y/[n])")
            start_input = time.time()
            flag_continue = True
            while flag_continue and time.time() > response_time:
                elapsed_time = time.time() - start_input

                if elapsed_time >= node.user_response_time:
                    node.get_logger().info("\nTimeout reached. Exiting.")
                    if flag_action > 48:
                        node.get_logger().info('land')
                        action.land()
                        flag_continue = False
                    else:
                        node.get_logger().info('base')
                        try:
                            action.go_to_base()
                        except Exception:
                            action.land()
                        flag_continue = False

                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    user_input = input().lower()
                    response_time = time.time() + node.user_response_time
                    node.get_logger().info(f"User response: {user_input}")
                    if user_input == 'y':
                        node.get_logger().info("Continuing...")
                        flag_continue = False
                    elif user_input == 'n':
                        if flag_action > 48:
                            node.get_logger().info('land')
                            action.land()
                            flag_continue = False
                        else:
                            node.get_logger().info('base')
                            try:
                                action.go_to_base()
                            except Exception:
                                action.land()
                            flag_continue = False
                time.sleep(0.1)

        rclpy.spin_once(node, timeout_sec=1)

    rclpy.spin(node)

if __name__ == '__main__':
    listener()
