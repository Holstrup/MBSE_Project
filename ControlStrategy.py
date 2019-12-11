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

    def get_name(self):
        return self.name


class FifoControl(ControlStrategy):
    def __init__(self):

        """ Hyper Parameters """
        self.time_through_intersection = 20  # 20 steps = 2 seconds at step size 0.1
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
        self.vehicle_dict = {}
        self.right_turn_dict = {}
        self.inc_lanes = ["gneE2", "gneE4", "-gneE3", "-gneE5"]
        self.watch_dict = {"gneE2": [], "gneE4": [], "-gneE5": [], "-gneE3": []}
        self.right_turns = {'gneE2': '-gneE4', '-gneE5': '-gneE2', 'gneE4': 'gneE3', '-gneE3': 'gneE5'}
        self.straight_path = {'gneE2': 'gneE3', '-gneE3': '-gneE2', 'gneE4': 'gneE5', '-gneE5': '-gneE4'}
        self.que = []
        self.counter_dict = {}

    def register_vehicle(self, departed_id_list):
        for id in departed_id_list:
            # register vehicle in dict with the priority based on the desired path

            # Car is moving
            self.vehicle_dict[id] = True
            # Counter how long car has been at stop
            self.counter_dict[id] = 0

            # Set vehicle speedmode to 7
            traci.vehicle.setSpeedMode(id, 7)

    def update_vehicle_list(self):
        vehicles_to_delete = []
        for vehicle_id in self.vehicle_dict.keys():

            current_pos = traci.vehicle.getRoadID(vehicle_id)
            if current_pos not in self.inc_lanes:
                vehicles_to_delete.append(vehicle_id)

            # add to watch list if less than halfway to the junction or if que at lane
            elif self.dist_to_junction(vehicle_id) < 50 and vehicle_id not in self.watch_dict[current_pos]:

                self.watch_dict[current_pos].append(vehicle_id)

        for vehicle in vehicles_to_delete:
            inc_pos = traci.vehicle.getRoute(vehicle)[0]

            del self.vehicle_dict[vehicle]
            self.watch_dict[inc_pos].remove(vehicle)

    def remove_from_watch(self, road_id, vehicle_id):
        self.watch_dict[road_id].remove(vehicle_id)

    def get_vehicles(self):
        return self.vehicle_dict.keys()

    # function to check if vehicle is turning right
    def turning_right(self, vehicle_id):
        in_pos = traci.vehicle.getRoute(vehicle_id)[0]
        out_pos = traci.vehicle.getRoute(vehicle_id)[1]
        if self.right_turns[in_pos] == out_pos:
            return True
        else:
            # if either first or second if  True  then the car is not turning right
            return False

    def going_straight(self, vehicle_id):
        in_pos = traci.vehicle.getRoute(vehicle_id)[0]
        out_pos = traci.vehicle.getRoute(vehicle_id)[1]
        if self.straight_path[in_pos] == out_pos:
            return True
        else:
            return False

    def is_crossing(self, vehicle_id):
        if self.turning_right(vehicle_id) or self.going_straight(vehicle_id):
            return False
        else:
            return True

    # priority if cars are across from each other
    def car_path_priority(self, vehicle):
        if self.turning_right(vehicle):
            return 1
        elif self.going_straight(vehicle):
            return 2
        elif self.is_crossing(vehicle):
            return 3

    def dist_to_junction(self, vehicle_id):
        vehicle_pos = traci.vehicle.getPosition(vehicle_id)
        junction_pos = (0, 0)
        return math.sqrt((vehicle_pos[0] - junction_pos[0]) ** 2 + (vehicle_pos[1] - junction_pos[1]) ** 2)

    def time_to_junction(self, vehicle_id):

        dist = self.dist_to_junction(vehicle_id)
        speed = traci.vehicle.getSpeed(vehicle_id)
        return dist / (speed + 0.0001)

    def dist_to_stop(self, vehicle_id):
        speed = traci.vehicle.getSpeed(vehicle_id)
        dec = traci.vehicle.getDecel(vehicle_id)
        traveled_distance = speed ** 2 / (2 * dec)
        return traveled_distance

    def get_watch_list(self):
        return self.watch_dict.keys()

    '''def get_sorted_watch_list(self):
        #return list of [(key,value),(key2,value2)] where value is lowest
        return sorted(self.watch_dict.items(), key=lambda kv: kv[1])'''

    def is_to_the_right(self, vehicle1, vehicle2):
        vehicle1_pos = traci.vehicle.getRoadID(vehicle1)
        vehicle2_pos = traci.vehicle.getRoadID(vehicle2)
        index_vehicle1 = self.inc_lanes.index(vehicle1_pos)
        index_vehicle2 = self.inc_lanes.index(vehicle2_pos)
        if index_vehicle1 + 1 == index_vehicle2 or (index_vehicle1 == 3 and index_vehicle2 == 0):
            return True
        else:
            return False

    def cars_to_the_right(self, vehicle):
        vehicle_pos = traci.vehicle.getRoadID(vehicle)
        index_vehicle = self.inc_lanes.index(vehicle_pos)

        # find pos to the right

        if index_vehicle == 3:
            pos_right = self.inc_lanes[0]
        else:
            pos_right = self.inc_lanes[index_vehicle + 1]

        if len(self.watch_dict[pos_right]) >= 1:
            return True
        else:
            return False

    def is_across(self, vehicle1, vehicle2):
        vehicle1_pos = traci.vehicle.getRoadID(vehicle1)
        vehicle2_pos = traci.vehicle.getRoadID(vehicle2)

        index_vehicle1 = self.inc_lanes.index(vehicle1_pos)
        index_vehicle2 = self.inc_lanes.index(vehicle2_pos)

        to_the_left = (index_vehicle1 - 1 == index_vehicle2 or (index_vehicle1 == 0 and index_vehicle2 == 3))
        if not (self.is_to_the_right(vehicle1, vehicle2) and to_the_left):
            return True
        else:
            return False

    def is_to_the_left(self, vehicle1, vehicle2):
        vehicle1_pos = traci.vehicle.getRoadID(vehicle1)
        vehicle2_pos = traci.vehicle.getRoadID(vehicle2)

        index_vehicle1 = self.inc_lanes.index(vehicle1_pos)
        index_vehicle2 = self.inc_lanes.index(vehicle2_pos)

        return (index_vehicle1 - 1 == index_vehicle2 or (index_vehicle1 == 0 and index_vehicle2 == 3))

    def will_collide(self, vehicle1, vehicle2):
        time_to_turn = 2
        # if 2 vehicles have less than x time between them
        time_to_intersection1 = self.time_to_junction(vehicle1)
        time_to_intersection2 = self.time_to_junction(vehicle2)

        if abs(time_to_intersection1 - time_to_intersection2) < time_to_turn:
            return True
        else:
            return False

    def form_vehicle_dict(self, vehicle_list):
        dict = {}
        for vehicle in vehicle_list:
            dist_to_junc = self.dist_to_junction(vehicle)
            dict[vehicle] = [dist_to_junc, self.can_stop(vehicle)]
        return dict

    def is_prioritized(self, vehicle1, vehicle2):

        prioritized = True

        if self.cars_to_the_right(vehicle1):
            return False
        # checks if vehicle across is going straight or right
        elif self.is_across(vehicle1, vehicle2):
            vehicle1_prio = self.car_path_priority(vehicle1)
            vehicle2_prio = self.car_path_priority(vehicle2)
            if vehicle2_prio > vehicle1_prio:

                prioritized = False
            elif vehicle2_prio == vehicle1_prio and vehicle2_prio == 3:
                veh1_dist = self.dist_to_junction(vehicle1)
                veh2_dist = self.dist_to_junction(vehicle2)
                if veh1_dist > veh2_dist:
                    prioritized = False
        # if vehicle 2 is in the intersection
        elif traci.vehicle.getRoadID(vehicle2) not in self.inc_lanes:
            prioritized = False

        return prioritized

    def can_stop(self, vehicle):

        braking_dist = self.dist_to_stop(vehicle)
        dist_to_end_of_road = self.dist_to_junction(vehicle) - 25

        if braking_dist > dist_to_end_of_road:

            return False
        else:

            return True

    def stop_at_junction(self, vehicle_id):

        # from center of junction to end of road
        dist_from_center_to_end = 7.5
        current_pos = traci.vehicle.getRoadID(vehicle_id)

        traci.vehicle.setStop(vehicle_id, current_pos, 100 - dist_from_center_to_end)
        self.vehicle_dict[vehicle_id] = False

    def cancel_stop(self, vehicle_id):
        # from center of junction to end of road

        current_pos = traci.vehicle.getRoadID(vehicle_id)
        traci.vehicle.setSpeed(vehicle_id, -1)
        # traci.vehicle.setStop(vehicle_id,current_pos,100-7.5,duration=0)
        self.vehicle_dict[vehicle_id] = True
        self.counter_dict[vehicle_id] = 0

    def get_first_cars_in_que(self, road_id):
        qued_cars = []
        for each_inc_road in self.inc_lanes:
            if each_inc_road != road_id and len(self.watch_dict[each_inc_road]) >= 1:
                qued_cars.append(self.watch_dict[each_inc_road][0])
                if len(self.watch_dict[each_inc_road]) >= 2:
                    qued_cars.append(self.watch_dict[each_inc_road][1])
        return qued_cars

    def control(self, step):

        if traci.vehicle.getIDCount() > 0:

            # if any vehicles have departed in the given simulated step, add to vehicle list
            if traci.simulation.getDepartedNumber() > 0:
                self.register_vehicle(traci.simulation.getDepartedIDList())
            # update lists
            self.update_vehicle_list()

            for road_id in self.get_watch_list():

                vehicle_que = self.watch_dict[road_id]

                if len(vehicle_que) >= 1:
                    for vehicle in vehicle_que:

                        self.counter_dict[vehicle] += 1
                        other_vehicles = self.get_first_cars_in_que(road_id)
                        can_go = True

                        if len(other_vehicles) >= 1:
                            for other in other_vehicles:
                                if self.will_collide(vehicle, other):
                                    if not self.is_prioritized(vehicle, other):
                                        can_go = False


                                # both cars are stopped
                                elif traci.vehicle.getStopState(vehicle) == 1 and traci.vehicle.getStopState(
                                        other) == 1:

                                    if self.counter_dict[vehicle] < self.counter_dict[other]:
                                        can_go = False

                            if not can_go and self.vehicle_dict[vehicle] == True and self.can_stop(vehicle):

                                self.stop_at_junction(vehicle)
                            # If car is stopping but can go
                            elif self.vehicle_dict[vehicle] == False and can_go and traci.vehicle.getStopState(
                                    vehicle) == 0:

                                self.cancel_stop(vehicle)

                                # control.remove_from_watch(road_id,vehicle)
                            elif traci.vehicle.getStopState(vehicle) == 1 and can_go:

                                traci.vehicle.resume(vehicle)
                                self.vehicle_dict[vehicle] = False
                        else:
                            if traci.vehicle.getStopState(vehicle) == 0 and self.vehicle_dict[vehicle] == False:
                                self.cancel_stop(vehicle)

                            elif traci.vehicle.getStopState(vehicle) == 1:

                                traci.vehicle.resume(vehicle)
                                self.vehicle_dict[vehicle] = True
                                self.counter_dict[vehicle] = 0


class TlControl(ControlStrategy):

    def __init__(self):
        self.name = "Traffic Light Control"
        self.id = 2

        self.tl_period = 60
        self.priority_rl = 0
        self.priority_ud = 1
        self.min_distance = 40

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
                x, y = traci.vehicle.getPosition(vehicle_id)
                distance = math.sqrt(x ** 2 + y ** 2)

                if self.priority_ud == 0 and edge in getattr(ControlStrategy, 'edges_ud') and traci.vehicle.getRoadID(
                        vehicle_id) in getattr(ControlStrategy, 'incoming_edges') and self.min_distance < distance:
                    traci.vehicle.setStop(vehicle_id, edge, pos=90,
                                          duration=self.time_until_switch)  # set stop for vehicle
                    self.blocked_edges.append(edge)  # add edge to blocked edges
                elif self.priority_rl == 0 and edge in getattr(ControlStrategy, 'edges_lr') and traci.vehicle.getRoadID(
                        vehicle_id) in getattr(ControlStrategy, 'incoming_edges') and self.min_distance < distance:
                    traci.vehicle.setStop(vehicle_id, edge, pos=90,
                                          duration=self.time_until_switch)  # set stop for vehicle
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
                    if self.min_distance < distance < \
                            closest_vehicles[getattr(ControlStrategy, 'incoming_edges').index(edge)][1]:
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
        self.time_through_intersection = 0  # Seconds
        self.deceleration_parameter = 0  # 1 m/s
        self.time_history = 5000  # Seconds
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

                        dist = self.dist_from_junction(car)
                        velocity = dist / time_in
                        traci.vehicle.setSpeed(car, velocity)
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
                except Exception:
                    continue


class NoControl(ControlStrategy):

    def __init__(self):
        self.name = "No Control"
        self.id = 4

    def control(self, step):
        # put Grid code here
        pass
