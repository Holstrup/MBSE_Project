from __future__ import absolute_import
from __future__ import print_function

import sys
import random
import math


class Controller:

    def __init__(self):
        self.sumoBinary = ""
        self.config_path = ""
        self.num_steps = -1
        self.step_length = 0.0
        self.current_id = -1  # first ID is 0
        self.tl_period = -1  # green time per traffic light: 1 min = 60 s
        self.vehicle_appearance_probability = 0.0
        self.routes = ['du', 'dl', 'dr', 'ld', 'lr', 'lu', 'ul', 'ud', 'ur', 'ru', 'rl', 'rd']
        self.sm = 7
        self.priority_rl = 0
        self.priority_ud = 1
        self.route_edge_dict = {'du':'gneE2', 'dl':'gneE2', 'dr':'gneE2', 'ld':'-gneE5', 'lr':'-gneE5', 'lu':'-gneE5', 'ul':'-gneE3', 'ud':'-gneE3', 'ur':'-gneE3', 'ru':'gneE4', 'rl':'gneE4', 'rd':'gneE4'}

    def config(self, ns, sl, init_id, tlp, vap):
        self.num_steps = ns
        self.step_length = sl
        self.current_id = init_id  # first ID is 0
        self.tl_period = tlp  # green time per traffic light: 1 min = 60 s
        self.vehicle_appearance_probability = vap

    def run_for_alex(self, cmd):
        traci.start(cmd)

        for step in range(self.num_steps):
            traci.simulationStep()

            if step % 300 == 0:
                route_id = 'du'
                vehicle_id = "00000" + str(self.current_id)
                self.current_id = self.current_id + 1
                vehicle_id = vehicle_id[-6:]
                traci.vehicle.add(vehicle_id, route_id)
                traci.vehicle.setSpeed(vehicle_id, 10.0)
                traci.vehicle.setMinGap(vehicle_id, 1.0)
                traci.vehicle.setSpeedMode(vehicle_id, self.sm)
                traci.vehicle.setTau(vehicle_id, 0.0)
            elif step % 300 == 150:
                route_id = 'lr'
                vehicle_id = "00000" + str(self.current_id)
                self.current_id = self.current_id + 1
                vehicle_id = vehicle_id[-6:]
                traci.vehicle.add(vehicle_id, route_id)
                traci.vehicle.setSpeed(vehicle_id, 10.0)
                traci.vehicle.setMinGap(vehicle_id, 1.0)
                traci.vehicle.setSpeedMode(vehicle_id, self.sm)
                traci.vehicle.setTau(vehicle_id, 0.0)
                traci.vehicle.setColor(vehicle_id, (255, 0, 0))

        traci.close()

    def run(self, cmd):
        traci.start(cmd)
        switch = True
        time_until_switch = self.tl_period
        blocked_edges = []

        incoming_edges = ['gneE4', '-gneE5', 'gneE2', '-gneE3']
        edges_ud = ['gneE2', '-gneE3']
        edges_lr = ['gneE4', '-gneE5']

        for step in range(self.num_steps):
            traci.simulationStep()
            time_until_switch = time_until_switch - self.step_length

            # vehicle generator based on routes and probability
            if random.random() < self.vehicle_appearance_probability:
                route_id = self.routes[math.floor(random.random() * len(self.routes))]
                vehicle_id = "00000" + str(self.current_id)
                self.current_id = self.current_id + 1
                vehicle_id = vehicle_id[-6:]
                traci.vehicle.add(vehicle_id, route_id)
                # traci.vehicle.setSpeed(vehicle_id, 10.0) using setSpeed overrides the speed mode - not desirable
                traci.vehicle.setMinGap(vehicle_id, 3.0)
                traci.vehicle.setSpeedMode(vehicle_id, self.sm)
                traci.vehicle.setTau(vehicle_id, 0.0)
                traci.vehicle.setImperfection(vehicle_id, 0.0)
                # check if vehicle is on a lane with a red light and without a leading vehicle
                edge = self.route_edge_dict[route_id]
                if edge not in blocked_edges:
                    if self.priority_ud == 0 and edge in edges_ud:
                        traci.vehicle.setStop(vehicle_id, edge, pos=90, duration=time_until_switch)  # set stop for vehicle
                        blocked_edges.append(edge)  # add edge to blocked edges
                    elif self.priority_rl == 0 and edge in edges_lr:
                        traci.vehicle.setStop(vehicle_id, edge, pos=90, duration=time_until_switch)  # set stop for vehicle
                        blocked_edges.append(edge)  # add edge to blocked edges
            active_vehicles = traci.vehicle.getIDList()

            # detect traffic light switch
            if step * self.step_length % self.tl_period == 0:
                # change priority state
                switch = True
                tmp = self.priority_rl
                self.priority_rl = self.priority_ud
                self.priority_ud = tmp

            if switch:
                blocked_edges = []
                time_until_switch = self.tl_period

                # find closest vehicles to intersection's critical region
                closest_vehicles = [['-1', 100000.0], ['-1', 100000.0], ['-1', 100000.0], ['-1', 100000.0]]
                for vehicle in active_vehicles:
                    x, y = traci.vehicle.getPosition(vehicle)
                    edge = traci.vehicle.getRoadID(vehicle)
                    if edge in incoming_edges:
                        distance = math.sqrt(x ** 2 + y ** 2)
                        if 30.0 < distance < closest_vehicles[incoming_edges.index(edge)][1]:
                            closest_vehicles[incoming_edges.index(edge)] = [vehicle, distance]

                if self.priority_ud == 0:
                    vehicle_0 = closest_vehicles[incoming_edges.index(edges_ud[0])][0]
                    vehicle_1 = closest_vehicles[incoming_edges.index(edges_ud[1])][0]

                    if vehicle_0 != '-1':
                        edge = edges_ud[0]
                        try:
                            traci.vehicle.setStop(vehicle_0, edge, pos=90, duration=60)
                            blocked_edges.append(edge)
                        except:
                            print("Vehicle could not break and caused traci exception.")
                            print("Vehicle edge: " + edge)
                            print("Vehicle pos: " + str(traci.vehicle.getPosition(vehicle_0)))
                    if vehicle_1 != '-1':
                        edge = edges_ud[1]
                        try:
                            traci.vehicle.setStop(vehicle_1, edge, pos=90, duration=60)
                            blocked_edges.append(edge)
                        except:
                            print("Vehicle could not break and caused traci exception.")
                            print("Vehicle edge: " + edge)
                            print("Vehicle pos: " + str(traci.vehicle.getPosition(vehicle_1)))

                else:
                    vehicle_0 = closest_vehicles[incoming_edges.index(edges_lr[0])][0]
                    vehicle_1 = closest_vehicles[incoming_edges.index(edges_lr[1])][0]

                    if vehicle_0 != '-1':
                        edge = edges_lr[0]
                        try:
                            traci.vehicle.setStop(vehicle_0, edge, pos=90, duration=60)
                            blocked_edges.append(edge)
                        except:
                            print("Vehicle could not break and caused traci exception.")
                            print("Vehicle edge: " + edge)
                            print("Vehicle pos: " + str(traci.vehicle.getPosition(vehicle_0)))
                    if vehicle_1 != '-1':
                        edge = edges_lr[1]
                        try:
                            traci.vehicle.setStop(vehicle_1, edge, pos=90, duration=60)
                            blocked_edges.append(edge)
                        except:
                            print("Vehicle could not break and caused traci exception.")
                            print("Vehicle edge: " + edge)
                            print("Vehicle pos: " + str(traci.vehicle.getPosition(vehicle_1)))
            switch = False

        traci.close()

try:
    # Insert dir of sumo tools
    sys.path.append(
        'C:/Users/Bosse/Documents/00_DTU/01_Master/01_First_Semester/02223_Model-Based_Systems_Engineering/sumo-1.3.1/tools')
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

# Insert dir of sumo-gui
sumoBinary = "C:/Users/Bosse/Documents/00_DTU/01_Master/01_First_Semester/02223_Model-Based_Systems_Engineering/sumo-1.3.1/bin/sumo-gui.exe"

# Insert dir of config file
config_path = "C:/Users/Bosse/Documents/00_DTU/01_Master/01_First_Semester/02223_Model-Based_Systems_Engineering/networks/test.sumocfg"
import traci

controller = Controller()
controller.config(30000, 0.01, 0, 60, 0.004)
sumo_cmd = [sumoBinary, "-c", config_path, "--step-length", str(controller.step_length), "--full-output", "log-file.xml", "--verbose"]
controller.run(sumo_cmd)

