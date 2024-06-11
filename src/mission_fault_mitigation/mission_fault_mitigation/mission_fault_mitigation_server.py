#!/usr/bin/env python3
#@authors Veronica Vannini

# Import libraries
import pyAgrum as gum
import sys # Probably useless
import rclpy #ROS2
from rclpy.node import Node
from interfaces.srv import MissionFaultMitigation
from interfaces.msg import *
import math # Probably useless
# import rosnode, psutil # Probably useless
# from std_srvs.srv import Empty

#from rosplan_knowledge_msgs.srv import *
#from rosplan_knowledge_msgs.msg import *
#from rosplan_dispatch_msgs.msg import *
#from rosplan_dispatch_msgs.srv import *

#from mavros_msgs.msg import *
#from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix, Imu, BatteryState

from pylab import *
import matplotlib.pyplot as plt
import os
import json

from std_msgs.msg import String



# Classes
# Alterations made!!


class Plan(object):
    """
    The Plan class represents a plan in the system.

    This class creates a node named "plan_subscriber" and subscribes to the "rosplan_parsing_interface/complete_plan" topic.

    Attributes
    ----------
    sub : Node
        A ROS node named "plan_subscriber".
    plan : CompletePlan
        The current plan.

    Methods
    -------
    plan_callback(data)
        Updates the plan based on the received data and unsubscribes from the "rosplan_parsing_interface/complete_plan" topic.
    unsubscribe()
        Destroys the "plan_subscriber" node and shuts down rclpy.
    """
    def __init__(self):
        self.sub = rclpy.create_node("plan_subscriber")
        self.sub.create_subscription(CompletePlan, "rosplan_parsing_interface/complete_plan", self.plan_callback, 10)
        self.plan = CompletePlan()

    def plan_callback(self, data):
        self.plan = data
        self.unsubscribe()

    def unsubscribe(self):
        self.sub.destroy_node()
        rclpy.shutdown()

class Drone(object):
    """
    The Drone class represents a drone in the system.

    Attributes
    ----------
    sub : Subscription
        A ROS subscription to the 'mavros/global_position/global' topic.
    sub1 : Subscription
        A ROS subscription to the 'mavros/battery' topic.
    latitude : float
        The current latitude of the drone.
    longitude : float
        The current longitude of the drone.
    battery : float
        The current battery level of the drone.

    Methods
    -------
    global_position_callback(data)
        Updates the drone's latitude and longitude based on the received data and unsubscribes from the 'mavros/global_position/global' topic.
    battery_state_callback(data)
        Updates the drone's battery level based on the received data.
    unsubscribe()
        Unsubscribes from the 'mavros/global_position/global' topic.
    """
    def __init__(self):
        self.sub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.global_position_callback, 10)  
        self.sub1 = self.create_subscription(BatteryState, 'mavros/battery', self.battery_state_callback, 10)  
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

##############################

# Classe responsável pela ações do sistema.


class Region:
    """
    The Region class represents a geographical region.

    Attributes
    ----------
    idi : int
        The identifier of the region.
    name : str
        The name of the region.
    geo_points : list
        A list of geographical points defining the region.
    points : list
        A list of cartesian points defining the region.
    geo_center : tuple
        The geographical center of the region.
    cart_center : tuple
        The cartesian center of the region.

    Methods
    -------
    __init__(idi, name, geo_points, cart_points, geo_center, cart_center)
        Initializes a new instance of the Region class.
    """
    def __init__(self, idi, name, geo_points, cart_points, geo_center, cart_center):
        """
        Initializes a new instance of the Region class.

        Parameters
        ----------
        idi : int
            The identifier of the region.
        name : str
            The name of the region.
        geo_points : list
            A list of geographical points defining the region.
        cart_points : list
            A list of cartesian points defining the region.
        geo_center : tuple
            The geographical center of the region.
        cart_center : tuple
            The cartesian center of the region.
        """
        self.idi = idi
        self.name = name
        self.geo_points = geo_points
        self.points = cart_points
        self.geo_center = geo_center
        self.cart_center = cart_center


class CartesianPoint:
    """
    The CartesianPoint class represents a point in a 3D Cartesian coordinate system.

    Attributes
    ----------
    x : float
        The x-coordinate of the point.
    y : float
        The y-coordinate of the point.
    z : float
        The z-coordinate of the point. Defaults to 0.

    Methods
    -------
    __init__(x, y, z=0)
        Initializes a new instance of the CartesianPoint class.
    """
    def __init__(self, x, y, z=0):     
        """
        Initializes a new instance of the CartesianPoint class.

        Parameters
        ----------
        x : float
            The x-coordinate of the point.
        y : float
            The y-coordinate of the point.
        z : float, optional
            The z-coordinate of the point. Defaults to 0.
        """
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"[{self.x}, {self.y}, {self.z}]"


class GeoPoint:
    """
    The GeoPoint class represents a point on the Earth's surface.

    Attributes
    ----------
    latitude : float
        The latitude of the point in degrees.
    longitude : float
        The longitude of the point in degrees.
    altitude : float
        The altitude of the point in meters.

    Methods
    -------
    __init__(latitude, longitude, altitude)
        Initializes a new instance of the GeoPoint class.
    """
    def __init__(self, latitude, longitude, altitude):
        """
        Initializes a new instance of the GeoPoint class.

        Parameters
        ----------
        latitude : float
            The latitude of the point in degrees.
        longitude : float
            The longitude of the point in degrees.
        altitude : float
            The altitude of the point in meters.
        """
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude


#Calcula a probabilidade de chegar em uma regiao X dado que a bateria restante nela é Y (lembrando que a bateria é inconsistente abaixo de 15%)
def dist_prob(bateria):
    """
    Calculates the probability based on the battery level.

    This function returns a probability value based on the input battery level. The returned probability is higher when the battery level is higher.

    Parameters
    ----------
    bateria : int or float
        The battery level.

    Returns
    -------
    float
        The probability corresponding to the battery level.
    """
    if(5 <= bateria and bateria < 15):
        return 0.20
    elif(15 <= bateria and bateria < 30):
        return 0.50
    elif(30 <= bateria and bateria < 60):
        return 0.75
    elif(60 <= bateria and bateria < 80):
        return 0.85
    elif(80 <= bateria and bateria < 100):
        return 0.95
    else:
        return 0

#Rede Bayesiana definida no sistema que utiliza:
## Bateria inicial, consumo definido à partir da bateria,
## ArrayActions (vetor de objeto da classe ação) e as probabilidades definidas
def calc_probabilities(bateria_init, consumo, ArrayActions, prob) :
    """
    Calculates the probabilities for each action in the mission based on the battery level.

    This function iterates over the ArrayActions, and for each action, it calculates the remaining battery level and the probability of successfully completing the action. If the action is to recharge the battery, the battery level is set to 100 and the probability is set to 1.

    Parameters
    ----------
    bateria_init : int or float
        The initial battery level.
    consumo : int or float
        The battery consumption per unit of action duration.
    ArrayActions : list
        A list of actions. Each action is an object with properties 'name' and 'duration'.
    prob : list
        A list to which the calculated probabilities will be appended.

    Returns
    -------
    list
        The updated list of probabilities.
    """
    bateria = []
    bateria.append(bateria_init)

    # Indica se é a primeira região ou não
    FLAG = 0
    print("\n")
    print("---------------------- PROBABILITY INFO --------------------------")
    print("\n")
    
    # Para cada regiao
    for obj in ArrayActions[:-1]:
        # se a regiao é inicial
        if(obj.name == "recharge_battery" or obj.name == "recharge_input"):
        # if(obj.name == "recharge_battery"):
            bateria.append(100)
            print("Bateria recarregada")
            prob.append(1)
            print()
        # senao
        else:
        # calcula a prob e a bateria restante para chegar a proxima regiao
            bateria.append(bateria[len(bateria)-1] - consumo*float(obj.duration))
            prob.append(dist_prob(bateria[len(bateria) - 1]))
            print("Probabilidade de " + get_DecisionString(obj) + " : " + str(prob[len(prob)-1]))
            print("bateria restante = " + str(bateria[len(bateria)-1]))
            print()
    return prob

#Essa funcao determina se a ação acontece ou não, se seu valor for 0.1, significa que ações com 0.1 de probabilidade iriam contar como (SIM)
def value(valor, action):
    """
    Determines if an action can be performed based on a given value.

    This function checks if the provided value is above a certain threshold. The threshold depends on the action. For 'go_to_base', 'clean_camera', and 'recharge_input' actions, the threshold is 0.2. For all other actions, the threshold is 0.5.

    Parameters
    ----------
    valor : float
        The value to be checked.
    action : str
        The action to be performed.

    Returns
    -------
    int
        Returns 1 if the action can be performed (i.e., the value is above the threshold), and 0 otherwise.
    """
    if(action == 'go_to_base' or action == 'clean_camera' or action == 'recharge_input'):
        if(valor >= 0.2):
            return 1
        else:
            return 0
    else:
        if(valor >= 0.5):
            return 1;
        else:
            return 0;

def get_DecisionString(obj):
    """
    Generates a decision string based on the name and parameters of an object.

    This function checks if the name of the object contains "go_to". If it does, it returns a string in the format 'go_to_' followed by the value of the second parameter of the object. Otherwise, it returns a string in the format of the object's name followed by '_' and the value of the first parameter of the object.

    Parameters
    ----------
    obj : object
        The object for which to generate the decision string. The object should have a 'name' attribute and a 'parameters' attribute, which is a list of objects with a 'value' attribute.

    Returns
    -------
    str
        The generated decision string.
    """
    if "go_to" in obj.name:
        return  'go_to_'+ str(obj.parameters[1].value)
    else:
        return  obj.name +'_'+ str(obj.parameters[0].value)

def log(replan, bn):
    """
    Logs the replan and Bayesian network (bn) to a JSON file and a .net file respectively.

    This function appends the replan to the 'replan' field of the last entry in the mission_log.json file. It also saves the Bayesian network (bn) to a .net file with a unique name in the 'net' directory. The index of the .net file is also appended to the 'bn_net' field of the last entry in the mission_log.json file.

    Parameters
    ----------
    replan : int
        The replan value to be logged.
    bn : gum.BayesNet
        The Bayesian network to be logged.

    """
    # log path
    LOG_PATH = "~/harpia/results/"
    LOG_PATH = os.path.expanduser(LOG_PATH)
    log_json = LOG_PATH + "mission_log.json"
    log_net = LOG_PATH + "net/"

    # open log
    with open(log_json, "r") as log_file:
            log_file = json.load(log_file)

    # add replan
    log_file[-1]['replan'] = log_file[-1]['replan'] + replan

    i = 0
    while os.path.exists(log_net+"%s.net" % i):
        i += 1
    gum.saveBN(bn, log_net+"%s.net" % i)
    # with open("WaterSprinkler.net","r") as out:
    #   print(out.read())

    log_file[-1]['bn_net'].append(i)
    # dump log
    with open(log_json, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)
    outfile.close()

def mission_fault_mitigation(req):
    """
    Performs fault mitigation for a mission based on the provided request.

    This function creates a plan for a drone mission and calculates the probabilities of successfully completing each action in the plan based on the drone's battery level and discharge rate. It uses a Bayesian network for fault detection. If the probability of successfully completing an action is below a certain threshold, the function triggers a replanning and returns a response with a flag indicating that replanning is needed.

    Parameters
    ----------
    req : Request
        The request object containing the initial action id, and the drone's battery information including the discharge and recharge rates.

    Returns
    -------
    MissionFaultMitigationResponse
        The response object containing a flag indicating whether replanning is needed (1 if replanning is needed, 0 otherwise).
    """
    p = Plan()
    uav = Drone()
    while p.plan.plan ==  []:
        print("Waiting for Complete Plan...")
        rclpy.spin_once(self.sub, timeout_sec=1)

    ArrayActions = p.plan.plan[req.action_id:]

    print("\n")
    print("------------------------- PATH INFO ------------------------------")
    for obj in ArrayActions[:-1]:
        print(get_DecisionString(obj), obj.duration,  sep = " ")

    while uav.battery ==  float("inf"):
            print("Waiting for UAV battery...")
            rclpy.spin_once(self.sub, timeout_sec=1)

    print("\n")
    print("----------------------- HARDWARE INFO ----------------------------")
    #define bateria e consumo inicial
    bateria_init = uav.battery

    #Define qual o tipo de bateria (AQUI SERIAM MUDADOS PARA OUTROS TIPOS DE BATERIA À PARTIR DE OUTROS IF's)
    consumo = req.uav.battery.discharge_rate #0.042 #hardware["discharge-rate-battery"]    
    recarga = req.uav.battery.recharge_rate #3.5   #hardware["recharge-rate-battery"]  
    print("Discharge rate battery = " + str(consumo))
    print("Battery ammount = " + str(bateria_init)) 

    # creating BN
    bn = gum.BayesNet('FaultDetection')
    print(bn)

    #define os conjuntos de modelos e probabilidades
    prob = []
    model = []
    # lista de inferencia
    # ie = []

    #flag do sistema -> 0 se não precisa replan
    FLAG = 0

    #Cria um contador para as regioes / separa região inicial do resto
    count = 0

    #Pega os valores das probabilidades definidas na rede
                         #  current batt, uav discharge rate, plan, prob
    prob = calc_probabilities(bateria_init, consumo, ArrayActions, prob)

    dict_evidences = {}

    print("\n")
    print("---------------------- PREDICTION INFO ---------------------------")

    relaxed_action = ['go_to_base', 'clean_camera', 'recharge_input']

    for obj in ArrayActions[:-1]:
        #Criando acoes
        # StringA = 'actions_' + str(count)
        StringDecisao = str(count)+"_"+get_DecisionString(obj)
        # action = gum.LabelizedVariable(StringA,'.',2)
        action = gum.LabelizedVariable(StringDecisao,'.',2)
        action.changeLabel(0,"Replan") # false
        action.changeLabel(1, "Do_Action") # true
        node = bn.add(action)
        model.append(action)
        if(count):
            bn.addArc(node-1, node)
            bn.cpt(node)[:]=[ [1,.0], [abs(1 - prob[count]),abs(prob[count])]]
        else:
            bn.cpt(node).fillWith([abs(1 - prob[count]),abs(prob[count])])

        ie=gum.LazyPropagation(bn)
        # ie = gum.InfluenceDiagramInference(model[count])

        ie.setEvidence(dict_evidences)
        
        ie.makeInference()
        if(count):
            inf = ie.posterior(node)
            ie_list = inf.tolist()
            # print(ie.H(count))
            # if(ie.H(count) < 0.15):
            # if(ie_list[0] == 1):
            if((obj.name in relaxed_action and ie_list[1] >= .2) or ie_list[1] >= ie_list[0]):
                print()
                print('--------------------- Atualmente em '+str(count+1)+' ---------------------')
                print('Probabilidade: '+str(ie_list[1]))
                dict_evidences[StringDecisao] = 1 #value(prob[count], obj.name)
            else:
                print("\n------------------------- REPLANNING -----------------------------\n")
                FLAG = 1
                # log(1, bn)#erro em salvar
                return MissionFaultMitigationResponse(FLAG)
        else:
            ie.eraseAllEvidence()


        count += 1

        
    # gnb.showInference(bn, evs={})
    # gum.saveBN(bn,"WaterSprinkler.net")
    # with open("WaterSprinkler.net","r") as out:
    #   print(out.read())
    #log(0, bn) #erro em salvar
    return MissionFaultMitigationResponse(FLAG)

def mission_fault_mitigation_server():
    """
    Initializes a server for the mission fault mitigation service.

    This function initializes a ROS2 node named 'mission_fault_mitigation_server' and creates a service named 'harpia/mission_fault_mitigation' of type MissionFaultMitigation.\n
    The service uses the mission_fault_mitigation function to handle requests.\n
    After the service is created, the function logs a message indicating that the service is ready and then spins the node to keep it running.

    """
    rclpy.init()
    node = rclpy.create_node('mission_fault_mitigation_server')
    srv = node.create_service(MissionFaultMitigation, 'harpia/mission_fault_mitigation', mission_fault_mitigation)
    node.get_logger().info('Mission Fault Mitigation Ready')
    rclpy.spin(node)

if __name__ == '__main__':
    mission_fault_mitigation_server()

def main():
    mission_fault_mitigation_server()
    