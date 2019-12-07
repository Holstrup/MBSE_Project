import random
import math
import libraries.traci as traci


class TrafficGenerator:
    def __init__(self, vap=0.0, routes=[], tau=0.0, speed_mode=7, imperfection=0.0, min_gap=2.0):
        self.current_id = 0
        self.routes = routes

        self.vehicle_appearance_probability = vap
        self.tau = tau
        self.sm = speed_mode
        self.imperfection = imperfection
        self.min_gap = min_gap
        self.initial_speed = 13.8

    def generate_traffic_flow(self):
        # vehicle generator based on routes and probability
        if random.random() < self.vehicle_appearance_probability:
            route_id = self.routes[math.floor(random.random() * len(self.routes))]
            vehicle_id = "000000" + str(self.current_id)
            self.current_id = self.current_id + 1
            vehicle_id = vehicle_id[-6:]
            traci.vehicle.add(vehicle_id, route_id, departSpeed=self.initial_speed)
            traci.vehicle.setMinGap(vehicle_id, self.min_gap)
            traci.vehicle.setSpeedMode(vehicle_id, self.sm)
            traci.vehicle.setTau(vehicle_id, self.tau)
            traci.vehicle.setImperfection(vehicle_id, self.imperfection)
