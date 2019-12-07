from __future__ import absolute_import
from __future__ import print_function
import sys

""" Configuration code """
step_length = 0.1
run_config = "sumo-gui" # "sumo" or "sumo-gui"
LOW = 0.026
HIGH = 0.042

vehicle_appearance_probability = LOW
speed_mode = 7
current_id = 1
routes = ['du', 'dl', 'dr', 'ld', 'lr', 'lu', 'ul', 'ud', 'ur', 'ru', 'rl', 'rd']


try:
#Insert dir of sumo tools
    sys.path.append('/usr/local/Cellar/sumo/1.3.1/share/sumo/tools')
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

#Insert dir of sumo-gui
sumoBinary = "/usr/local/Cellar/sumo/1.3.1/bin/" + run_config

#Insert dir of config file
sumoCmd = [sumoBinary, "-c", "networks/test.sumocfg", '--step-length', str(step_length), '--full-output', "log-file.xml", "--verbose"]


""" Main code """
import math
import traci
import traci.constants as tc
from SystemController import SystemController
from SystemControllerFutureV2 import SystemController as FutureIsHere
from Control_Logic import ControlLogic2
import random

traci.start(sumoCmd, label="sim1")
sim1=traci.getConnection("sim1")


#controller = SystemController(step_length)
controller = FutureIsHere(step_length)



def generate_traffic():
    global speed_mode, current_id, routes, vehicle_appearance_probability
    if random.random() < vehicle_appearance_probability:
        route_id = routes[math.floor(random.random() * len(routes))]

        vehicle_id = "00000" + str(current_id)
        current_id += 1
        vehicle_id = vehicle_id[-6:]

        traci.vehicle.add(vehicle_id, route_id, departSpeed = 13.8)
        traci.vehicle.setMinGap(vehicle_id, 3.0)
        traci.vehicle.setSpeedMode(vehicle_id, speed_mode)
        traci.vehicle.setTau(vehicle_id, 0.0)
        traci.vehicle.setImperfection(vehicle_id, 0.0)

for step in range(3000):
    sim1.simulationStep()
    generate_traffic()
    controller.update_state(step)

traci.close()
