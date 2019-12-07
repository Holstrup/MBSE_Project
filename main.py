import config_traci
from Control_Logic import ControlLogic2

import traci
import traci.constants as tc
import random
import math

traci.start(config_traci.sumoCmd, label="sim1")
sim1=traci.getConnection("sim1")
control = ControlLogic2(sim1)

LOW = 0.026
HIGH = 0.042

vehicle_appearance_probability = LOW
sm = 7
current_id = 1
routes = ['du', 'dl', 'dr', 'ld', 'lr', 'lu', 'ul', 'ud', 'ur', 'ru', 'rl', 'rd']

def generate_traffic():
    global sm, current_id, routes, vehicle_appearance_probability
    if random.random() < vehicle_appearance_probability:
        route_id = routes[math.floor(random.random() * len(routes))]

        vehicle_id = "00000" + str(current_id)
        current_id += 1
        vehicle_id = vehicle_id[-6:]

        traci.vehicle.add(vehicle_id, route_id, departSpeed = 13.8)
        traci.vehicle.setMinGap(vehicle_id, 3.0)
        traci.vehicle.setSpeedMode(vehicle_id, sm)
        traci.vehicle.setTau(vehicle_id, 0.0)
        traci.vehicle.setImperfection(vehicle_id, 0.0)

for step in range(3000):

    sim1.simulationStep()
    generate_traffic()

    control.run_sim()

traci.close()

