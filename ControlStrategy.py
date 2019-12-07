import libraries.traci as traci
import math
import numpy as np
from LaneDefinitions import paths


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

    def set_step_length(self, step_length):
        self.step_length = step_length


class FifoControl(ControlStrategy):
    def __init__(self):

        """ Hyper Parameters """
        self.time_through_intersection = 40  # 20 steps = 2 seconds at step size 0.1
        self.deceleration_parameter = 0  # 1 m/s

        """ System Variables """
        self.vehicle_stack = []
        self.inc_lanes = ["-gneE3_0", "gneE4_0", "gneE2_0", "-gneE5_0"]  # Find way to make this logic dynamic
        self.out_lanes = ["gneE3_0", "-gneE4_0", "-gneE2_0", "gneE5_0"]  # Find way to make this logic dynamic
        self.reserved_slots = []
        self.vehicle_reservations = {}

        self.name = "FIFO Control"
        self.id = 0

    def time_from_junction(self, car, junction_pos=(0, 0)):
        """
        :param car: vehicle id
        :param junction_pos: position of the junction in x,y coordinates
        :return: ETA of car to intersection
        """
        dist = self.dist_from_junction(car)
        speed = traci.vehicle.getSpeed(car)
        if speed == 0:
            return dist / (speed + 0.0001)
        else:
            return dist / speed

    def dist_from_junction(self, car, junction_pos=(0, 0)):
        vehicle_pos = traci.vehicle.getPosition(car)
        return math.sqrt((vehicle_pos[0] - junction_pos[0]) ** 2 + (vehicle_pos[1] - junction_pos[1]) ** 2) - 7.5

    def update_id_list(self):
        """
        Updates the vehicle stack. Incoming vehicles are also assigned speed mode 0
        """
        for arrived in traci.simulation.getArrivedIDList():
            self.vehicle_stack.remove(arrived)
            try:
                self.reserved_slots = [x for x in self.reserved_slots if x not in self.vehicle_reservations[arrived]]
                del self.vehicle_reservations[arrived]
            except KeyError:
                print("-- Move along --")

        for departed in traci.simulation.getDepartedIDList():
            self.vehicle_stack.append(departed)

    def schedule_arrival(self, car, step, time_in, time_out):
        """
        :param car: vehicle id
        :param time_in: ETA car in intersection
        :param time_out: ETA car out of intersection
        """

        time_in = int(round(time_in, -1))
        time_out = int(round(time_out, -1))
        reserved_times = list(range(time_in, time_out + 1, 10))

        if car not in self.vehicle_reservations:
            if len((set(reserved_times).intersection(set(self.reserved_slots)))) == 0:

                # Reserve time for car
                self.vehicle_reservations[car] = reserved_times
                self.reserved_slots.extend(reserved_times)
                traci.vehicle.setSpeed(car, traci.vehicle.getSpeed(car))
            else:
                # Find next available time for vehicle
                target_time = max(self.reserved_slots) + 10

                # Calculate average speed for it to get there on that specific time
                target_speed = self.dist_from_junction(car) / (target_time - step) * 10

                # Set new speed - something
                traci.vehicle.setSpeed(car, target_speed - self.deceleration_parameter)

                # Reserve time for car
                reserved_times = list(range(target_time, target_time + self.time_through_intersection, 10))
                self.vehicle_reservations[car] = reserved_times
                self.reserved_slots.extend(reserved_times)

    def control(self, step):
        self.update_id_list()

        if len(self.vehicle_reservations) == 0:
            pass

        if step % 5 == 0:
            print(step, self.vehicle_reservations)

            # For all cars
            for car in self.vehicle_stack:
                # If the car is on it's way into the intersection
                if traci.vehicle.getLaneID(car) in self.inc_lanes:
                    time = self.time_from_junction(car)
                    self.schedule_arrival(car, step, step + time * 10,
                                          step + time * 10 + self.time_through_intersection)

                # If the car is on its way out of the intersection, set max speed.
                elif traci.vehicle.getLaneID(car) in self.out_lanes:
                    laneId = traci.vehicle.getLaneID(car)
                    traci.vehicle.setSpeed(car, traci.lane.getMaxSpeed(laneId))

                # If the car is in the intrersection, set the speed to max speed
                else:
                    laneId = traci.vehicle.getLaneID(car)
                    traci.vehicle.setSpeed(car, traci.lane.getMaxSpeed(laneId))


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

        """ Hyper Parameters """
        self.time_through_intersection = 4  # Seconds
        self.deceleration_parameter = 0  # 1 m/s
        self.time_history = 1200  # Seconds
        self.junction_size = 2  # Grid Size

        """ System Variables """
        self.vehicle_stack = []
        self.inc_lanes = ["-gneE3_0", "gneE4_0", "gneE2_0", "-gneE5_0"]  # Find way to make this logic dynamic
        self.out_lanes = ["gneE3_0", "-gneE4_0", "-gneE2_0", "gneE5_0"]  # Find way to make this logic dynamic
        self.vehicle_reservations = {}
        self.lane_times = {"-gneE3_0": 0, "gneE4_0": 0, "gneE2_0": 0, "-gneE5_0": 0}

        # X,Y,T
        self.space_time = np.zeros((self.junction_size, self.junction_size, self.time_history)).astype(bool)

    def time_from_junction(self, car, junction_pos=(0, 0)):
        """
        :param car: vehicle id
        :param junction_pos: position of the junction in x,y coordinates
        :return: ETA of car to intersection
        """
        dist = self.dist_from_junction(car)
        speed = traci.vehicle.getSpeed(car)
        if speed == 0:
            return dist / (speed + 0.0001)
        else:
            return dist / speed

    def dist_from_junction(self, car, junction_pos=(0, 0)):
        vehicle_pos = traci.vehicle.getPosition(car)
        return math.sqrt((vehicle_pos[0] - junction_pos[0]) ** 2 + (vehicle_pos[1] - junction_pos[1]) ** 2) - 7.5

    def update_id_list(self):
        """
        Updates the vehicle stack. Incoming vehicles are also assigned speed mode 0
        """
        for arrived in traci.simulation.getArrivedIDList():
            self.vehicle_stack.remove(arrived)
            try:
                del self.vehicle_reservations[arrived]
            except KeyError:
                print("-- Move along --")

        for departed in traci.simulation.getDepartedIDList():
            self.vehicle_stack.append(departed)

    def compatible_path(self, path_mask, time_in, time_out):
        for i in range(time_in, time_out, 1):
            full_sum = np.add(path_mask, self.space_time[:, :, int(time_in + i)])
            if np.any((full_sum > 1)):
                return False
        return True

    def update_space_time(self):
        """
        Refreshes the space time array for a new timestep.
        Index 0 in the time direction is removed, and a new empty array is added
        at index self.time_history.
        """
        self.space_time = np.delete(self.space_time, obj=0, axis=2)
        self.space_time = np.dstack((self.space_time, np.zeros((self.junction_size, self.junction_size))))

    def intersection_available(self, time_in, time_out, bitmap):
        time_in = math.floor(time_in)
        time_out = math.ceil(time_out)

        for i in range(time_in, time_out, 1):
            full_sum = np.add(bitmap, self.space_time[:, :, int(time_in + i)])
            if np.any((full_sum > 1)):
                return False
        return True

    def findNextAvailability(self, time_in, time_in_min, bitmap):
        if time_in_min > 0:
            time = time_in_min
        else:
            time = time_in

        time = math.ceil(time)

        for new_time in range(time, self.time_history - self.time_through_intersection, 1):
            if self.intersection_available(new_time, new_time + self.time_through_intersection, bitmap):
                return new_time, new_time + self.time_through_intersection

    def reserveGrid(self, time_in_steps, car, path, step, car_lane):
        # Book time
        for i in range(self.time_through_intersection):
            self.space_time[:, :, int(time_in_steps + i)] = np.add(self.space_time[:, :, int(time_in_steps + i)], path)

        self.vehicle_reservations[car] = (time_in_steps, time_in_steps + self.time_through_intersection * 10)
        self.lane_times[car_lane] = time_in_steps / 10 + self.time_through_intersection
        return None

    def control(self, step):
        if len(self.vehicle_stack) == 0:
            pass

        self.update_space_time()
        self.update_id_list()

        for car in self.vehicle_stack:
            # If the car is on it's way into the intersection
            if traci.vehicle.getLaneID(car) in self.inc_lanes:
                time_in = self.time_from_junction(car)
                time_out = time_in + self.time_through_intersection
                car_lane = traci.vehicle.getLaneID(car)
                car_route = traci.vehicle.getRouteID(car)
                time_in_min = self.lane_times[car_lane]

                if time_in < 100:
                    if car not in self.vehicle_reservations.keys():
                        bitmap = paths[car_route]
                        if not self.intersection_available(time_in, time_out, bitmap) or time_in < time_in_min:
                            time_in, time_out = self.findNextAvailability(time_in, time_in_min, bitmap)

                        self.reserveGrid(time_in * 10, car, bitmap, step, car_lane)


            # If the car is on its way out of the intersection, set max speed.
            elif traci.vehicle.getLaneID(car) in self.out_lanes:
                laneId = traci.vehicle.getLaneID(car)
                traci.vehicle.setSpeed(car, traci.lane.getMaxSpeed(laneId))

            # If the car is in the intersection, set the speed to max speed
            else:
                try:
                    laneId = traci.vehicle.getLaneID(car)
                    traci.vehicle.setSpeed(car, traci.lane.getMaxSpeed(laneId))
                except traci.exceptions:
                    continue




class NoControl(ControlStrategy):

    def __init__(self):
        self.name = "No Control"
        self.id = 4

    def control(self, step):
        # put Grid code here
        pass
