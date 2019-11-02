import math
import traci
import traci.constants as tc

class SystemController:
    def __init__(self, simulation):

        """ Hyper Parameters """
        self.time_through_intersection = 20  # 20 steps = 2 seconds at step size 0.1
        self.deceleration_parameter = 1  # 1 m/s


        """ System Variables """
        self.simulation = simulation
        self.vehicle_stack = []
        self.inc_lanes = ["-gneE3_0", "gneE4_0", "gneE2_0", "-gneE5_0"] # Find way to make this logic dynamic
        self.reserved_slots = []
        self.vehicle_reservations = {}



    def time_from_junction(self, car, junction_pos=(0, 0)):
        """
        :param car: vehicle id
        :param junction_pos: position of the junction in x,y coordinates
        :return: ETA of car to intersection
        """
        dist = self.dist_from_junction(car)
        speed = traci.vehicle.getSpeed(car)
        return dist / (speed + 0.0001)


    def dist_from_junction(self, car, junction_pos=(0, 0)):
        vehicle_pos = traci.vehicle.getPosition(car)
        return math.sqrt((vehicle_pos[0] - junction_pos[0]) ** 2 + (vehicle_pos[1] - junction_pos[1]) ** 2)

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
            traci.vehicle.setSpeedMode(departed, 00)

    def schedule_arrival(self, car, step, time_in, time_out):
        """
        :param car: vehicle id
        :param time_in: ETA car in intersection
        :param time_out: ETA car out of intersection
        """

        time_in = int(round(time_in,-1))
        time_out = int(round(time_out, -1))
        reserved_times = list(range(time_in, time_out + 1, 10))

        if car not in self.vehicle_reservations:
            if len((set(reserved_times).intersection(set(self.reserved_slots)))) == 0:
                if car == "f02.0": print("Car managed to find correct speed")
                self.vehicle_reservations[car] = reserved_times
                self.reserved_slots.extend(reserved_times)
                traci.vehicle.setSpeed(car, traci.vehicle.getSpeed(car))
            else:
                # Find next available time for vehicle
                target_time = max(self.reserved_slots) + 10
                print("----")
                print("Trouble is brewing for " + car, target_time)

                # Calculate average speed for it to get there on that specific time
                target_speed = self.dist_from_junction(car) / (target_time - step) * 10

                # Set new speed - something
                traci.vehicle.setSpeed(car, target_speed - self.deceleration_parameter)

                # Set new time slot
                print("----")
        print(self.reserved_slots)




    def update_state(self, step):
        if len(self.vehicle_stack) == 0:
            pass

        # For all cars
        for car in self.vehicle_stack:
            # If the car is on it's way into the intersection
            if traci.vehicle.getLaneID(car) in self.inc_lanes:
                time = self.time_from_junction(car)
                self.schedule_arrival(car, step, step + time * 10, step + time * 10 + self.time_through_intersection)




