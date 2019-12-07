import math
import traci
import traci.constants as tc
import numpy as np
from LaneDefinitions import paths
import math

class SystemController:
    def __init__(self, step_length):

        """ Hyper Parameters """
        self.time_through_intersection = 4  # Seconds
        self.deceleration_parameter = 0  # 1 m/s
        self.time_history = 1200 # Seconds
        self.junction_size = 2 # Grid Size


        """ System Variables """
        self.vehicle_stack = []
        self.inc_lanes = ["-gneE3_0", "gneE4_0", "gneE2_0", "-gneE5_0"] # Find way to make this logic dynamic
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
        if time_in_min > 0: time = time_in_min
        else: time = time_in

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

    def update_state(self, step):
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
                        if not self.intersection_available(time_in,time_out,bitmap) or time_in < time_in_min:
                            time_in, time_out = self.findNextAvailability(time_in, time_in_min, bitmap)

                        self.reserveGrid(time_in*10,car,bitmap,step,car_lane)


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


