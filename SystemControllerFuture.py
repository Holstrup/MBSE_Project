import math
import traci
import traci.constants as tc
import numpy as np
from LaneDefinitions import paths
import math

class SystemController:
    def __init__(self, step_length, speedMode = 0):

        """ Hyper Parameters """
        self.time_through_intersection = 30  # 20 steps = 2 seconds at step size 0.1
        self.deceleration_parameter = 1  # 1 m/s
        self.step_length = step_length
        self.speedMode = speedMode
        self.update_freq = 1 * (1 / step_length) # Every second
        self.time_history = 1200 # Seconds
        self.junction_size = 2


        """ System Variables """
        self.vehicle_stack = []
        self.inc_lanes = ["-gneE3_0", "gneE4_0", "gneE2_0", "-gneE5_0"] # Find way to make this logic dynamic
        self.out_lanes = ["gneE3_0", "-gneE4_0", "-gneE2_0", "gneE5_0"]  # Find way to make this logic dynamic
        self.reserved_slots = []
        self.vehicle_reservations = {}

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
        return math.sqrt((vehicle_pos[0] - junction_pos[0]) ** 2 + (vehicle_pos[1] - junction_pos[1]) ** 2) - 7


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
            traci.vehicle.setSpeedMode(departed, self.speedMode)


    def next_available_time(self, car, time, path_mask, step):

        for i in range(int(time), int(time + 50), 1):
            if self.compatible_path(path_mask, i):
                target_time = i
                break

        try:
            new_speed = (self.dist_from_junction(car) / target_time) - self.deceleration_parameter
            new_speed = math.ceil(new_speed)
        except UnboundLocalError:
            print("ERROR WITH " + car)

        return new_speed, target_time

    def schedule_arrival(self, car, time, step):
        """
        :param car: vehicle id
        :param time: ETA car in intersection
        """

        path = paths[traci.vehicle.getRouteID(car)[1:]]

        if not self.compatible_path(path, time):
            # Path not compatible at time -> Find next compatible time
            print("Old speed ", traci.vehicle.getSpeed(car))
            new_speed, time = self.next_available_time(car, time, path, step)
            print("New speed ", new_speed)
            print("Distance  ", self.dist_from_junction(car))
            print("Time      ", self.dist_from_junction(car) / new_speed)
            # Set new speed
            traci.vehicle.setSpeed(car, new_speed)


        # Book time
        through_time = self.time_through_intersection // 10 + 1
        for i in range(through_time):
            self.space_time[:, :, int(time + i)] = np.add(self.space_time[:, :, int(time + i)], path)

        self.vehicle_reservations[car] = (time, step + int(time) * 10)


    def compatible_path(self, path_mask, time_from_now):
        through_time = self.time_through_intersection // 10
        for i in range(through_time):
            full_sum = np.add(path_mask, self.space_time[:, :, int(time_from_now + i)])
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
        self.space_time = np.dstack((self.space_time, np.ones((self.junction_size, self.junction_size))))

    def update_state(self, step):
        if len(self.vehicle_stack) == 0:
            pass

        self.update_space_time()
        print(self.space_time[:, :, 0])
        print(self.vehicle_reservations)

        for car in self.vehicle_stack:
            # If the car is on it's way into the intersection
            if traci.vehicle.getLaneID(car) in self.inc_lanes:
                time = self.time_from_junction(car)
                if time > 55:
                    print("time from", time, car)
                else:
                    if car not in self.vehicle_reservations.keys():
                        self.schedule_arrival(car, round(time, 2), step)

            # If the car is on its way out of the intersection, set max speed.
            elif traci.vehicle.getLaneID(car) in self.out_lanes:
                laneId = traci.vehicle.getLaneID(car)
                traci.vehicle.setSpeed(car, traci.lane.getMaxSpeed(laneId))

            # If the car is in the intersection, set the speed to max speed
            else:
                laneId = traci.vehicle.getLaneID(car)
                traci.vehicle.setSpeed(car, traci.lane.getMaxSpeed(laneId))


