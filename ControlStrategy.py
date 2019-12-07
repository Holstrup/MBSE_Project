import libraries.traci as traci
import math


class ControlStrategy:
    routes = ['du', 'dl', 'dr', 'ld', 'lr', 'lu', 'ul', 'ud', 'ur', 'ru', 'rl', 'rd']
    route_edge_dict = {'du': 'gneE2', 'dl': 'gneE2', 'dr': 'gneE2', 'ld': '-gneE5', 'lr': '-gneE5',
                       'lu': '-gneE5', 'ul': '-gneE3', 'ud': '-gneE3', 'ur': '-gneE3', 'ru': 'gneE4',
                       'rl': 'gneE4', 'rd': 'gneE4'}
    incoming_edges = ['gneE4', '-gneE5', 'gneE2', '-gneE3']
    edges_ud = ['gneE2', '-gneE3']
    edges_lr = ['gneE4', '-gneE5']

    def control(self, step):
        pass


class FifoControl(ControlStrategy):

    def __init__(self):
        self.name = "FIFO Control"
        self.id = 0

    def control(self, step):
        # put FIFO code here
        pass


class RhpControl(ControlStrategy):

    def __init__(self):
        self.name = "Right Hand Precedence Control"
        self.id = 1

    def control(self, step):
        # put RHP code here
        pass


class TlControl(ControlStrategy):

    def __init__(self):
        self.name = "Traffic Light Control"
        self.id = 2

        self.tl_period = 60
        self.priority_rl = 0
        self.priority_ud = 1

        self.switch = True
        self.time_until_switch = self.tl_period
        self.blocked_edges = []

        self.step_length = -0.1

    def set_step_length(self, step_length):
        self.step_length = step_length

    def control(self, step):
        self.time_until_switch = self.time_until_switch - self.step_length

        # check if a vehicle was generated on a lane with a red light and without a leading vehicle
        active_vehicles = traci.vehicle.getIDList()
        if active_vehicles:
            vehicle_id = active_vehicles[len(active_vehicles) - 1]
            route_id = traci.vehicle.getRouteID(active_vehicles[len(active_vehicles) - 1])
            edge = self.route_edge_dict[route_id]
            if edge not in self.blocked_edges:
                if self.priority_ud == 0 and edge in getattr(ControlStrategy, 'edges_ud'):
                    traci.vehicle.setStop(vehicle_id, edge, pos=90, duration=self.time_until_switch)  # set stop for vehicle
                    self.blocked_edges.append(edge)  # add edge to blocked edges
                elif self.priority_rl == 0 and edge in getattr(ControlStrategy, 'edges_lr'):
                    traci.vehicle.setStop(vehicle_id, edge, pos=90, duration=self.time_until_switch)  # set stop for vehicle
                    self.blocked_edges.append(edge)  # add edge to blocked edges

        # detect traffic light switch
        if step * self.step_length % self.tl_period == 0:
            # change priority state
            self.switch = True
            tmp = self.priority_rl
            self.priority_rl = self.priority_ud
            self.priority_ud = tmp

        if self.switch and active_vehicles:
            self.blocked_edges = []
            self.time_until_switch = self.tl_period

            # find closest vehicles to intersection's critical region
            closest_vehicles = [['-1', 100000.0], ['-1', 100000.0], ['-1', 100000.0], ['-1', 100000.0]]
            for vehicle in active_vehicles:
                x, y = traci.vehicle.getPosition(vehicle)
                edge = traci.vehicle.getRoadID(vehicle)
                if edge in getattr(ControlStrategy, 'incoming_edges'):
                    distance = math.sqrt(x ** 2 + y ** 2)
                    if 30.0 < distance < closest_vehicles[getattr(ControlStrategy, 'incoming_edges').index(edge)][1]:
                        closest_vehicles[getattr(ControlStrategy, 'incoming_edges').index(edge)] = [vehicle, distance]

            if self.priority_ud == 0:
                vehicle_0 = closest_vehicles[getattr(ControlStrategy, 'incoming_edges').index(
                    getattr(ControlStrategy, 'edges_ud')[0])][0]
                vehicle_1 = closest_vehicles[getattr(ControlStrategy, 'incoming_edges').index(
                    getattr(ControlStrategy, 'edges_ud')[1])][0]

                if vehicle_0 != '-1':
                    edge = getattr(ControlStrategy, 'edges_ud')[0]
                    try:
                        traci.vehicle.setStop(vehicle_0, edge, pos=90, duration=self.tl_period)
                        self.blocked_edges.append(edge)
                    except:
                        print("Vehicle could not break and caused traci exception.")
                        print("Vehicle edge: " + edge)
                        print("Vehicle pos: " + str(traci.vehicle.getPosition(vehicle_0)))
                if vehicle_1 != '-1':
                    edge = getattr(ControlStrategy, 'edges_ud')[1]
                    try:
                        traci.vehicle.setStop(vehicle_1, edge, pos=90, duration=self.tl_period)
                        self.blocked_edges.append(edge)
                    except:
                        print("Vehicle could not break and caused traci exception.")
                        print("Vehicle edge: " + edge)
                        print("Vehicle pos: " + str(traci.vehicle.getPosition(vehicle_1)))

            else:
                vehicle_0 = closest_vehicles[getattr(ControlStrategy, 'incoming_edges').index(
                    getattr(ControlStrategy, 'edges_lr')[0])][0]
                vehicle_1 = closest_vehicles[getattr(ControlStrategy, 'incoming_edges').index(
                    getattr(ControlStrategy, 'edges_lr')[1])][0]

                if vehicle_0 != '-1':
                    edge = getattr(ControlStrategy, 'edges_lr')[0]
                    try:
                        traci.vehicle.setStop(vehicle_0, edge, pos=90, duration=self.tl_period)
                        self.blocked_edges.append(edge)
                    except:
                        print("Vehicle could not break and caused traci exception.")
                        print("Vehicle edge: " + edge)
                        print("Vehicle pos: " + str(traci.vehicle.getPosition(vehicle_0)))
                if vehicle_1 != '-1':
                    edge = getattr(ControlStrategy, 'edges_lr')[1]
                    try:
                        traci.vehicle.setStop(vehicle_1, edge, pos=90, duration=self.tl_period)
                        self.blocked_edges.append(edge)
                    except:
                        print("Vehicle could not break and caused traci exception.")
                        print("Vehicle edge: " + edge)
                        print("Vehicle pos: " + str(traci.vehicle.getPosition(vehicle_1)))
        self.switch = False


class GridControl(ControlStrategy):

    def __init__(self):
        self.name = "Grid Control"
        self.id = 3

    def control(self, step):
        # put Grid code here
        pass


class NoControl(ControlStrategy):

    def __init__(self):
        self.name = "No Control"
        self.id = 4

    def control(self, step):
        # put Grid code here
        pass
